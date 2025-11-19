#!/usr/bin/env python3
import os
from pathlib import Path
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatusArray, GoalStatus

import inspect

class RecordOnExecute(Node):
    def __init__(self):
        super().__init__('record_on_execute')

        # (1) 패키지/작업 경로 파라미터를 먼저 선언
        self.declare_parameter('package_name', '')   # 예: 'kist_robot_config', 'bipedal_config'
        self.declare_parameter('workspace_dir', '')  # 절대경로 직접 지정시 사용

        # (2) 우선순위에 따라 workspace_dir 결정
        pkg_override = self.get_parameter('package_name').get_parameter_value().string_value
        ws_override  = self.get_parameter('workspace_dir').get_parameter_value().string_value

        if ws_override:
            workspace_dir = Path(ws_override).expanduser().resolve()
        elif pkg_override:
            workspace_dir = (Path.home() / pkg_override).resolve()
        else:
            # 기존 fallback: 이 노드가 속한 패키지명(=python_tools 등)으로 홈 아래 생성
            pkg_name = __name__.split('.')[0]
            if pkg_name == '__main__':
                this_file = Path(inspect.getfile(RecordOnExecute))
                pkg_name = this_file.parent.parent.name
            workspace_dir = (Path.home() / pkg_name).resolve()

        workspace_dir.mkdir(parents=True, exist_ok=True)
        self.workspace_dir = workspace_dir  # 필요하면 다른 메서드에서 사용

        # (3) 기본 CSV 경로를 최종 workspace_dir 기반으로 설정
        default_csv_path = str(workspace_dir / "joint_log.csv")

        # (4) 나머지 파라미터
        self.declare_parameter('controller_name', 'upper_body_controller')
        self.declare_parameter('csv_path', default_csv_path)

        self.controller_name = self.get_parameter('controller_name').get_parameter_value().string_value
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value

        self.latest_js = None
        self.recorded_goal_ids = set()

        status_topic = f'/{self.controller_name}/follow_joint_trajectory/_action/status'
        self.status_sub = self.create_subscription(GoalStatusArray, status_topic, self.on_status, 10)
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.on_js, 50)

        # 경로 보장 + 파일 없으면 touch
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        if not os.path.exists(self.csv_path):
            open(self.csv_path, 'a').close()
            self.get_logger().info(f"[record_on_execute] (touch) created: {self.csv_path}")

        self.get_logger().info(f"[record_on_execute] listening: {status_topic}")
        self.get_logger().info(f"[record_on_execute] will append to CSV: {self.csv_path}")

    # 가장 최근 joint_states 덮어쓰기
    def on_js(self, msg: JointState):
        self.latest_js = msg

    # 액션 상태 콜백
    def on_status(self, msg: GoalStatusArray):
        # status_list 안에 여러 goal의 상태가 들어옴
        for st in msg.status_list:
            gid = st.goal_info.goal_id.uuid  # bytes(16)
            gid_key = bytes(gid)

            if st.status == GoalStatus.STATUS_SUCCEEDED and gid_key not in self.recorded_goal_ids:
                self.recorded_goal_ids.add(gid_key)
                self.record_snapshot()

    # CSV에 스냅샷 기록
    def record_snapshot(self):
        if self.latest_js is None:
            self.get_logger().warn("No JointState yet; skipping CSV write.")
            return

        names = list(self.latest_js.name)
        positions = list(self.latest_js.position) if self.latest_js.position else []
        if not positions or len(names) != len(positions):
            self.get_logger().warn("JointState has empty/mismatched positions; skipping.")
            return

        file_exists = os.path.exists(self.csv_path)
        file_empty = (os.path.getsize(self.csv_path) == 0) if file_exists else True

        timestamp = datetime.now().isoformat(timespec='seconds')
        row = [timestamp] + positions

        try:
            with open(self.csv_path, mode='a', newline='') as f:
                writer = csv.writer(f)
                if file_empty:
                    header = ['timestamp'] + names
                    writer.writerow(header)
                writer.writerow(row)
            self.get_logger().info(
                f"Recorded {len(positions)} joints to CSV at execute-complete: {self.csv_path}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV: {e}")
            
def main():
    rclpy.init()
    node = RecordOnExecute()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
