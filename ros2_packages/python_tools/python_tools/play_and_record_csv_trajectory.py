#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import csv
from pathlib import Path
from datetime import datetime
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

import inspect


def try_parse_iso(ts: str) -> Optional[datetime]:
    try:
        return datetime.fromisoformat(ts)
    except Exception:
        return None


class PlayAndRecordCsvTrajectory(Node):
    
    def __init__(self):
        super().__init__('play_and_record_csv_trajectory')

        from pathlib import Path
        import inspect

        self.startup_ok = True

        # (A) 먼저 패키지/작업경로 관련 파라미터 선언
        self.declare_parameter('package_name', '')   # 예: 'kist_robot_config' 또는 'bipedal_config'
        self.declare_parameter('workspace_dir', '')  # 절대경로를 직접 주고 싶을 때

        # (B) workspace_dir 결정 로직 (패키지명 → *_workspace 매핑)
        pkg_override = self.get_parameter('package_name').get_parameter_value().string_value.strip()
        ws_override  = self.get_parameter('workspace_dir').get_parameter_value().string_value.strip()

        def pkg_to_workspace_name(pname: str) -> str:
            mapping = {
                'bipedal_config': 'bipedal_workspace',
                'kist_robot_config': 'kist_robot_workspace',
            }
            return mapping.get(pname, f"{pname}_workspace") if pname else ''

        if ws_override:
            # 1순위: 사용자가 명시한 절대/상대 경로
            workspace_dir = Path(ws_override).expanduser().resolve()
        elif pkg_override:
            # 2순위: 패키지명 기반으로 홈 디렉터리 아래 매핑
            workspace_name = pkg_to_workspace_name(pkg_override)
            workspace_dir = (Path.home() / workspace_name).resolve()
        else:
            # 3순위: 이 노드가 속한 패키지명(보통 python_tools) → *_workspace
            pkg_name = __name__.split('.')[0]
            if pkg_name == '__main__':
                this_file = Path(inspect.getfile(PlayAndRecordCsvTrajectory))
                pkg_name = this_file.parent.parent.name
            workspace_name = pkg_to_workspace_name(pkg_name)
            workspace_dir = (Path.home() / workspace_name).resolve()

        workspace_dir.mkdir(parents=True, exist_ok=True)

        # 기본 CSV 경로 (모션 입력/샘플 출력)
        default_motion_csv = workspace_dir / "joint_log.csv"
        default_sample_csv = workspace_dir / "joint_sample.csv"

        # ===== 파라미터 (플레이) =====
        self.declare_parameter('csv_path', str(default_motion_csv))               # 입력 CSV (재생용)
        self.declare_parameter('controller_name', 'upper_body_controller')        # 액션 서버 네임스페이스
        self.declare_parameter('joint_names', [])                                  # 비어 있으면 CSV 헤더 사용
        self.declare_parameter('wait_server_timeout', 5.0)
        self.declare_parameter('goal_time_tolerance', 0.5)
        self.declare_parameter('start_lead', 0.1)
        self.declare_parameter('velocity_fallback', -1.0)  # (미사용, 유지)
        self.declare_parameter('min_increment', 0.001)
        self.declare_parameter('default_dt', 0.2)

        # 시간 처리 옵션
        self.declare_parameter('use_csv_timestamps', False)   # CSV의 시간 간격 그대로 사용 여부
        self.declare_parameter('uniform_dt', 2.0)             # 균등 간격(초)
        self.declare_parameter('max_dt', 2.0)                 # CSV 모드 최대 간격 상한(초)

        # ===== 파라미터 (기록) =====
        self.declare_parameter('sample_dt', 0.10)                                 # 기록 간격(추천: 0.05~0.10)
        self.declare_parameter('sample_csv_path', str(default_sample_csv))

        # ===== 속도 계산(필수) =====
        self.declare_parameter('ema_alpha', 0.35)  # EMA 스무딩 계수(0.2~0.5 추천)

        # ===== 파라미터 로드 =====
        self.csv_path: str = self.get_parameter('csv_path').value
        self.controller_name: str = self.get_parameter('controller_name').value
        self.user_joint_names: List[str] = self.get_parameter('joint_names').value
        self.wait_server_timeout: float = float(self.get_parameter('wait_server_timeout').value)
        self.goal_time_tolerance: float = float(self.get_parameter('goal_time_tolerance').value)
        self.start_lead: float = float(self.get_parameter('start_lead').value)
        self.velocity_fallback: float = float(self.get_parameter('velocity_fallback').value)  # (미사용)
        self.min_increment: float = float(self.get_parameter('min_increment').value)
        self.default_dt: float = float(self.get_parameter('default_dt').value)

        self.use_csv_timestamps: bool = bool(self.get_parameter('use_csv_timestamps').value)
        self.uniform_dt: float = float(self.get_parameter('uniform_dt').value)
        self.max_dt: float = float(self.get_parameter('max_dt').value)

        self.sample_dt: float = float(self.get_parameter('sample_dt').value)
        self.sample_csv_path: str = self.get_parameter('sample_csv_path').value
        self.ema_alpha: float = float(self.get_parameter('ema_alpha').value)

        # ===== 액션 클라이언트 =====
        action_ns = f'/{self.controller_name}/follow_joint_trajectory'
        self.client = ActionClient(self, FollowJointTrajectory, action_ns)

        # ===== /joint_states 구독 =====
        self.latest_js: Optional[JointState] = None
        self.create_subscription(JointState, '/joint_states', self._on_js, 50)

        # ===== 기록 제어 =====
        self.sampling_active: bool = False
        self.sample_timer = self.create_timer(self.sample_dt, self._on_sample_timer)
        self.sample_start_steady = None  # steady time (sec)
        self.file_header_names: List[str] = []   # 헤더에 기록된 joint 이름들(positions 기준)
        self._wrote_header: bool = False
        self._try_load_existing_header()

        # (로그용) Trajectory 조인트 목록
        self.traj_joint_names: List[str] = []

        # 유한차분 상태
        self._diff_last_pos: Optional[List[float]] = None
        self._diff_last_t: Optional[float] = None
        self._ema_vel: Optional[List[float]] = None

        self.get_logger().info(f"[play+record] Motion CSV: {self.csv_path}")
        self.get_logger().info(f"[play+record] Action server: {action_ns}")
        self.get_logger().info(f"[play+record] Sample CSV: {self.sample_csv_path} (dt={self.sample_dt:.3f}s)")
        self.get_logger().info(f"[play+record] use_csv_timestamps={self.use_csv_timestamps}, "
                               f"uniform_dt={self.uniform_dt}, max_dt={self.max_dt}")
        self.get_logger().info(f"[play+record] velocity := diff(position) + EMA (alpha={self.ema_alpha})")

        # 시작
        self._start()

    # =========================
    # /joint_states 콜백/헤더
    # =========================
    def _on_js(self, msg: JointState):
        self.latest_js = msg

    def _try_load_existing_header(self):
        """기존 CSV가 있으면 헤더에서 pos: 컬럼을 파싱해 joint 이름 목록을 복원."""
        try:
            if os.path.exists(self.sample_csv_path):
                with open(self.sample_csv_path, newline='') as f:
                    reader = csv.reader(f)
                    first = next(reader, None)
                if first and len(first) >= 2:
                    cols = first[2:]
                    names = [c[4:] for c in cols if c.startswith('pos:')]
                    if names:
                        self.file_header_names = names
                        self._wrote_header = True
                        self.get_logger().info(f"[play+record] Existing sample header joints: {self.file_header_names}")
        except Exception as e:
            self.get_logger().warn(f"[play+record] Failed to read existing header: {e}")

    def _ensure_sample_header(self, names: List[str]):
        if self._wrote_header:
            return
        os.makedirs(os.path.dirname(self.sample_csv_path), exist_ok=True)
        with open(self.sample_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp', 't_rel'] + [f'pos:{n}' for n in names] + [f'vel:{n}' for n in names]
            writer.writerow(header)
        self._wrote_header = True
        self.file_header_names = list(names)
        self.get_logger().info(f"[play+record] Sample header written: {self.file_header_names}")

    # =========================
    # 기록 타이머
    # =========================
    def _on_sample_timer(self):
        if not self.sampling_active:
            return
        js = self.latest_js
        if js is None or not js.name or not js.position:
            return

        names = list(js.name)
        positions = list(js.position)
        if not positions or len(names) != len(positions):
            self.get_logger().warn_throttle(5.0, "[play+record] JointState empty/mismatched; skipping.")
            return

        # header 생성(positions 기준 이름)
        if not self._wrote_header:
            self._ensure_sample_header(names)

        # 이름 불일치 시 스킵(컬럼 정합 유지)
        if names != self.file_header_names:
            self.get_logger().warn_throttle(
                5.0, "[play+record] JointState.name != file header; skipping sample."
            )
            return

        # === 속도 계산: joint_states.velocity는 무시, position으로 직접 계산 ===
        t_now = self.get_clock().now().nanoseconds * 1e-9

        if (self._diff_last_pos is not None) and (self._diff_last_t is not None):
            dt = max(1e-4, t_now - self._diff_last_t)  # 0 나눗셈 방지
            raw_vel = [(p - lp) / dt for p, lp in zip(positions, self._diff_last_pos)]
            # EMA 스무딩
            if self._ema_vel is None or len(self._ema_vel) != len(raw_vel):
                self._ema_vel = list(raw_vel)
            else:
                a = self.ema_alpha
                self._ema_vel = [a*rv + (1-a)*ev for rv, ev in zip(raw_vel, self._ema_vel)]
            velocities = list(self._ema_vel)
        else:
            # 첫 샘플은 직전 데이터가 없으므로 0으로 시작
            velocities = [0.0] * len(names)

        # 다음 차분을 위한 상태 업데이트
        self._diff_last_pos = list(positions)
        self._diff_last_t = t_now

        # ===== CSV 기록 =====
        now_iso = datetime.now().isoformat(timespec='seconds')
        t_rel = 0.0
        if self.sample_start_steady is not None:
            t_rel = (self.get_clock().now().nanoseconds * 1e-9) - self.sample_start_steady

        try:
            with open(self.sample_csv_path, 'a', newline='') as f:
                csv.writer(f).writerow([now_iso, f"{t_rel:.3f}"] + positions + velocities)
        except Exception as e:
            self.get_logger().error(f"[play+record] Failed to write sample CSV: {e}")

    # =========================
    # 실행 플로우
    # =========================
    def _start(self):
        if not self.client.wait_for_server(timeout_sec=self.wait_server_timeout):
            self.get_logger().error("FollowJointTrajectory action server not available.")
            self.startup_ok = False
            return

        try:
            traj = self._load_csv_to_trajectory(self.csv_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load motion CSV: {e}")
            self.startup_ok = False            
            return

        self.traj_joint_names = list(traj.joint_names)

        now = self.get_clock().now()
        lead = Duration(seconds=self.start_lead)
        traj.header.stamp = (now + lead).to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = DurationMsg(
            sec=int(self.goal_time_tolerance),
            nanosec=int((self.goal_time_tolerance % 1.0) * 1e9)
        )

        self.get_logger().info(
            f"[play+record] Sending trajectory ({len(traj.points)} pts). "
            f"Start lead={self.start_lead:.2f}s; traj joints={self.traj_joint_names}"
        )
        send_future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback):
        # 이번 버전에서는 피드백 속도를 사용하지 않음(전적으로 계산값 기록)
        pass

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server.")
            rclpy.shutdown()
            return

        self.get_logger().info("[play+record] Goal accepted. Start sampling...")
        self.sampling_active = True
        self.sample_start_steady = self.get_clock().now().nanoseconds * 1e-9

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result_ready)

    def _on_result_ready(self, future):
        self.sampling_active = False
        try:
            result_wrap = future.result()
            r = result_wrap.result
            code = getattr(r, 'error_code', None)
            msg = getattr(r, 'error_string', '')
        except Exception as e:
            self.get_logger().error(f"Failed to get result: {e}")
            rclpy.shutdown()
            return

        code_map = {
            0: "SUCCESSFUL",
            -1: "INVALID_GOAL",
            -2: "INVALID_JOINTS",
            -3: "OLD_HEADER",
            -4: "PATH_TOLERANCE_VIOLATED",
            -5: "GOAL_TOLERANCE_VIOLATED",
        }
        human = code_map.get(code, f"UNKNOWN({code})")
        if code == 0:
            self.get_logger().info("[play+record] Trajectory execution SUCCEEDED.")
        else:
            self.get_logger().error(f"[play+record] Trajectory execution FAILED: {human} {msg}")

        self.get_logger().info(f"[play+record] Sampling finished. CSV at: {self.sample_csv_path}")
        rclpy.shutdown()

    # =========================
    # CSV -> JointTrajectory
    # =========================
    def _load_csv_to_trajectory(self, csv_path: str) -> JointTrajectory:
        with open(csv_path, newline='') as f:
            reader = csv.reader(f)
            rows = list(reader)

        if len(rows) < 2:
            raise RuntimeError("CSV must have a header and at least one data row.")

        header = rows[0]
        if len(header) < 2:
            raise RuntimeError("Header must include 'timestamp' and at least one joint name.")

        csv_joint_names = header[1:]

        # joint_names 확정 및 인덱스 맵
        if self.user_joint_names:
            joint_names = list(self.user_joint_names)
            missing = [j for j in joint_names if j not in csv_joint_names]
            if missing:
                raise RuntimeError(f"CSV missing joints: {missing}")
            idx_map = [csv_joint_names.index(j) for j in joint_names]
        else:
            joint_names = csv_joint_names
            idx_map = list(range(len(joint_names)))

        points: List[JointTrajectoryPoint] = []
        prev_ts: Optional[datetime] = None
        t_acc = 0.0
        last_good_dt: Optional[float] = None

        for r in rows[1:]:
            if len(r) < 1 + len(csv_joint_names):
                self.get_logger().warn(f"Skipping malformed row: {r}")
                continue

            # 포지션
            pos_all = r[1:]
            try:
                pos = [float(pos_all[i]) for i in idx_map]
            except Exception:
                self.get_logger().warn(f"Skipping row with non-float position(s): {r}")
                continue

            # === 시간 처리 ===
            if self.use_csv_timestamps:
                ts = try_parse_iso(r[0])
                if ts is not None and prev_ts is not None:
                    dt = (ts - prev_ts).total_seconds()
                else:
                    dt = self.uniform_dt  # 기본 간격 대체

                if dt <= 0:
                    dt = last_good_dt if (last_good_dt and last_good_dt > 0) else self.uniform_dt
                if dt > self.max_dt:
                    dt = self.max_dt
                if dt < self.min_increment:
                    dt = self.min_increment

                if ts is not None:
                    prev_ts = ts
                last_good_dt = dt
            else:
                dt = self.uniform_dt
                if dt < self.min_increment:
                    dt = self.min_increment

            t_acc += dt

            pt = JointTrajectoryPoint()
            pt.positions = pos
            # pt.velocities는 여기선 지정하지 않음(컨트롤러가 보간)

            sec = int(t_acc)
            nsec = int((t_acc - sec) * 1e9)
            pt.time_from_start = DurationMsg(sec=sec, nanosec=nsec)
            points.append(pt)

        if not points:
            raise RuntimeError("No valid trajectory points parsed from CSV.")

        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.points = points
        return traj



def main():
    rclpy.init()
    node = PlayAndRecordCsvTrajectory()
    if not getattr(node, "startup_ok", True):
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():   
            rclpy.shutdown()


if __name__ == '__main__':
    main()
