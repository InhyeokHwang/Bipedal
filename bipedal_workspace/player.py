#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import sys
import tty
import termios
import select
import can

from itertools import zip_longest
from canwrapper import CanWrapper
from robstride_motor import RobstrideMotor

# ===== 리스트 입력 (bipedal: 좌/우만) =====
from values_right import positions_list as RH_POSITIONS, velocities_list as RH_VELOCITIES
from values_left  import positions_list as LH_POSITIONS,  velocities_list as LH_VELOCITIES

# === 스위치 ===
ENABLE_CAN0 = True   # can0: 오른쪽
ENABLE_CAN1 = True   # can1: 왼쪽
DRY_RUN = False

# ==== 모터 배치 ====
# can0(오른쪽), can1(왼쪽) — 아래 리스트 길이 = CSV 열 개수와 일치해야 함
MOTOR_IDS_CAN0 = [2, 4, 6, 8]    # positions_list_0의 0,1,2,3열 (right)
MOTOR_IDS_CAN1 = [1, 3, 5, 7]    # positions_list_1의 0,1,2,3열 (left)

# === 제어 파라미터 (기본값) ===
TORQUE = 0.0
KP = 2.0
KD = 1.0
p = 2.4            # 플레이 속도 스케일(>1 빠름)
dt = 0.1 / p       # 스텝 dwell

# === 모터별 게인/토크 오버라이드 ===
KP_OVERRIDE = {
    6: 13.0,
    8: 18.0,
    5: 13.0,
    7: 18.0,
}
KD_OVERRIDE = {
    6: 8.0,
    8: 4.0,
    5: 8.0,
    7: 4.0,
}
TORQUE_OVERRIDE = {
    6: 2.5,
    8: 3.0,
    5: 2.5,
    7: 3.0,
}

def get_kp_for_id(motor_id: int, default: float) -> float:
    return KP_OVERRIDE.get(motor_id, default)

def get_kd_for_id(motor_id: int, default: float) -> float:
    return KD_OVERRIDE.get(motor_id, default)

def get_torque_for_id(motor_id: int, default: float) -> float:
    return TORQUE_OVERRIDE.get(motor_id, default)

# ===  모터별 각도 오프셋(rad) 오버라이드 ===
OFFSET_OVERRIDE = {
    2: math.radians(-4.0),
    # 3: math.radians(-14.0),
    # 4: math.radians(14.0),
    5: math.radians(10.0),
    6: math.radians(-24.0),
    7: math.radians(-9.0)
}

def apply_offsets(pos_lists_right, pos_lists_left,
                  motor_ids_right, motor_ids_left,
                  offset_table):
    """
    pos_lists_right : positions_list_0_raw (오른쪽 bus용, shape ~ [T][N_right])
    pos_lists_left  : positions_list_1_raw (왼쪽  bus용, shape ~ [T][N_left])
    motor_ids_right : MOTOR_IDS_CAN0
    motor_ids_left  : MOTOR_IDS_CAN1
    offset_table    : OFFSET_OVERRIDE (motor_id -> radians)

    return: (new_right, new_left) 오프셋 적용된 deep copy
    """
    new_right = []
    for row in pos_lists_right:
        new_row = []
        for col_idx, base_val in enumerate(row):
            m_id = motor_ids_right[col_idx]  # 이 열이 어떤 motor_id인지
            off = offset_table.get(m_id, 0.0)
            new_row.append(base_val + off)
        new_right.append(new_row)

    new_left = []
    for row in pos_lists_left:
        new_row = []
        for col_idx, base_val in enumerate(row):
            m_id = motor_ids_left[col_idx]
            off = offset_table.get(m_id, 0.0)
            new_row.append(base_val + off)
        new_left.append(new_row)

    return new_right, new_left

# --- Robstride 상태 변환/ID 규약 ---
MASTER_ID = 0x00  # 모터 펌웨어의 CAN_MASTER(보통 0x00)
P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -44.0, 44.0
T_MIN, T_MAX = -17.0, 17.0

# ---------- 리스트 구성/전처리 ----------
def concat_series(A, B):
    """행 길이가 달라도 안전하게 A행+B행을 이어붙인다."""
    out = []
    for a, b in zip_longest(A, B, fillvalue=[]):
        ra = list(a) if a else []
        rb = list(b) if b else []
        out.append(ra + rb)
    return out

# bipedal은 버스별로 합칠 파트가 없으니 그대로 사용
positions_list_0_raw  = RH_POSITIONS
velocities_list_0     = RH_VELOCITIES
positions_list_1_raw  = LH_POSITIONS
velocities_list_1     = LH_VELOCITIES

# ---- 리스트 유효성/형상 체크 ----
if not positions_list_0_raw or not positions_list_0_raw[0]:
    raise RuntimeError("오른쪽 positions_list가 비어 있거나 열이 없습니다.")
if not positions_list_1_raw or not positions_list_1_raw[0]:
    raise RuntimeError("왼쪽 positions_list가 비어 있거나 열이 없습니다.")

RH_W = len(positions_list_0_raw[0])
LH_W = len(positions_list_1_raw[0])

assert RH_W == len(MOTOR_IDS_CAN0), f"can0 열({RH_W}) != 모터({len(MOTOR_IDS_CAN0)})"
assert LH_W == len(MOTOR_IDS_CAN1), f"can1 열({LH_W}) != 모터({len(MOTOR_IDS_CAN1)})"

# --- 각도 오프셋 적용 ---
positions_list_0, positions_list_1 = apply_offsets(
    positions_list_0_raw,
    positions_list_1_raw,
    MOTOR_IDS_CAN0,
    MOTOR_IDS_CAN1,
    OFFSET_OVERRIDE
)

# 속도 스케일 (p배) - 시간축 스케일링과 일관되게
velocities_list_0 = [[v * p for v in row] for row in velocities_list_0]
velocities_list_1 = [[v * p for v in row] for row in velocities_list_1]

# ---------- 유틸 ----------
def _u16_to_float(x, x_min, x_max):
    return (x / 65535.0) * (x_max - x_min) + x_min

def parse_motor_feedback(msg):
    if msg is None or len(msg.data) < 8:
        return None
    data = bytes(msg.data)
    angle    = int.from_bytes(data[0:2], 'big')
    velocity = int.from_bytes(data[2:4], 'big')
    torque   = int.from_bytes(data[4:6], 'big')
    temp     = int.from_bytes(data[6:8], 'big')
    return {
        'angle':    _u16_to_float(angle,    P_MIN, P_MAX),
        'velocity': _u16_to_float(velocity, V_MIN, V_MAX),
        'torque':   _u16_to_float(torque,   T_MIN, T_MAX),
        'temp':     temp / 10.0
    }

def send_can(canif, arbitration_id, data=b'\x00'*8):
    """CanWrapper 래퍼 혹은 하위 bus로 송신."""
    try:
        if hasattr(canif, "send_message"):
            canif.send_message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        else:
            canif.bus.send(can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True))
    except Exception as e:
        print(f"[WARN] CAN 송신 실패: {e}")

def request_feedback(canif, motor_id, master_id=MASTER_ID):
    """Robstride 상태 요청(Type=0x02 확장 프레임) 1회 전송."""
    req_id = (0x02 << 24) | (master_id << 8) | motor_id
    send_can(canif, req_id, b'\x00' * 8)

def get_current_angle(canif, motor_id, timeout=0.4, master_id=MASTER_ID):
    """모터별 상태 요청을 날리고, 응답(Type=0x02, 해당 motor_id)만 파싱."""
    t_start = time.time()
    request_feedback(canif, motor_id, master_id=master_id)

    while time.time() - t_start < timeout:
        msg = canif.get_message(timeout=0.05)
        if not msg:
            continue
        rid    = msg.arbitration_id
        rtype  = (rid >> 24) & 0x3F
        rmotor = (rid >> 8)  & 0xFF
        if (getattr(msg, "is_extended_id", True) is False) or rtype != 0x02 or rmotor != motor_id:
            continue

        fb = parse_motor_feedback(msg)
        if fb and 'angle' in fb:
            ang = fb['angle']
            print(f"[FB] id={motor_id} angle={ang:.3f} rad")
            return ang
    raise RuntimeError(f"모터(id={motor_id}) 피드백 수신 실패")

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def safe_get(row, idx, default=0.0):
    return row[idx] if idx < len(row) else default

def _drain_stdin():
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if not r:
            break
        sys.stdin.read(1)

# ---------- 기존 wait_for_spacebar는 안 쓴다. 대신 posture hold하면서 기다리는 버전 ----------

# ★ NEW: 자세 유지하면서 space/q 기다리기
def hold_posture_until_space(motors0, motors1, hold_targets0, hold_targets1):
    """
    hold_targets0 / hold_targets1:
        run_startup_ramp 이후 로봇이 서야 하는 각도(target pose).
    이 함수는 그 포즈를 계속 m.send_operation_mode()로 유지시켜 주면서
    스페이스(시작)나 q(중단) 입력을 기다린다.
    """
    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        _drain_stdin()
        print("\n[READY] 스페이스바를 누르면 시작, Q는 중단 (HOLD MODE)")
        while True:
            # 키 입력 체크
            r, _, _ = select.select([sys.stdin], [], [], 0.01)
            if r:
                ch = sys.stdin.read(1)
                if ch == ' ':
                    print("[START]")
                    return True
                if ch in ('q', 'Q'):
                    print("[ABORT]")
                    return False

            # 현재 포즈 유지 명령 반복 송신
            for idx, m in enumerate(motors0):
                kp = get_kp_for_id(m.motor_id, 2.0)
                kd = get_kd_for_id(m.motor_id, 1.0)
                torque = get_torque_for_id(m.motor_id, 0.0)
                pos = hold_targets0[idx]
                if not DRY_RUN:
                    m.send_operation_mode(
                        pos=pos,
                        vel=0.0,
                        kp=kp,
                        kd=kd,
                        torque=torque
                    )

            for idx, m in enumerate(motors1):
                kp = get_kp_for_id(m.motor_id, 2.0)
                kd = get_kd_for_id(m.motor_id, 1.0)
                torque = get_torque_for_id(m.motor_id, 0.0)
                pos = hold_targets1[idx]
                if not DRY_RUN:
                    m.send_operation_mode(
                        pos=pos,
                        vel=0.0,
                        kp=kp,
                        kd=kd,
                        torque=torque
                    )

            time.sleep(0.01)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)

# ---------- 종료 후 선택 받는 함수 (기존 wait_for_choice 유지) ----------
def wait_for_choice(prompt="스페이스=종료, r=재시작, q=종료"):
    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        _drain_stdin()
        print(f"\n[READY] {prompt}")
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if ch == ' ':
                print("[CHOICE] 종료 선택")
                return "exit"
            if ch in ('r', 'R'):
                print("[CHOICE] 재시작 선택")
                return "restart"
            if ch in ('q', 'Q'):
                print("[CHOICE] 종료 선택(q)")
                return "exit"
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)

# ---------- 시작, 종료 복귀 & 정리 ----------
# === 트랙 첫 프레임으로 S-curve 램프-인 ===
def _s_curve(alpha: float) -> float:
    # 0~1에서 가속/감속이 부드러운 보간
    return 0.5 - 0.5*math.cos(math.pi * max(0.0, min(1.0, alpha)))

def _first_frame_for(bus_positions_list, fallback):
    """트랙 첫 프레임을 타깃으로. 없으면 fallback(현재각 등) 사용"""
    if bus_positions_list and bus_positions_list[0]:
        f0 = bus_positions_list[0]
        # 길이 정합은 호출부에서 슬라이스
        return f0[:]
    return fallback[:]

def run_startup_ramp(motors0, motors1, theta0, theta1,
                     n_step=300, hz=100.0):
    """
    현재각(theta*)에서 '트랙 첫 프레임'으로 S-curve로 이동.
    반환값: (target0, target1) = 실제로 선 포즈 (트랙 첫 프레임)
    """
    dt = 1.0 / hz

    # 트랙 첫 프레임(이미 OFFSET 적용된 positions_list_* 기준)
    target0_full = _first_frame_for(positions_list_0, theta0)
    target1_full = _first_frame_for(positions_list_1, theta1)

    # 모터 개수에 맞춰 잘라내기
    target0 = target0_full[:len(motors0)]
    target1 = target1_full[:len(motors1)]
    theta0  = theta0[:len(motors0)]
    theta1  = theta1[:len(motors1)]

    # 램프-인
    for step in range(n_step):
        a = (step + 1) / n_step
        sa = _s_curve(a)

        for i, m in enumerate(motors0):
            pos_des = theta0[i]*(1.0 - sa) + target0[i]*sa
            if not DRY_RUN:
                m.send_operation_mode(
                    pos=clamp(pos_des, P_MIN, P_MAX),
                    vel=0.0,
                    kp=get_kp_for_id(m.motor_id, 2.0),
                    kd=get_kd_for_id(m.motor_id, 1.0),
                    torque=get_torque_for_id(m.motor_id, 0.0)
                )

        for i, m in enumerate(motors1):
            pos_des = theta1[i]*(1.0 - sa) + target1[i]*sa
            if not DRY_RUN:
                m.send_operation_mode(
                    pos=clamp(pos_des, P_MIN, P_MAX),
                    vel=0.0,
                    kp=get_kp_for_id(m.motor_id, 2.0),
                    kd=get_kd_for_id(m.motor_id, 1.0),
                    torque=get_torque_for_id(m.motor_id, 0.0)
                )

        time.sleep(dt)

    print("\n[START] 모터 시작")
    # 이후 hold용 포즈로 트랙 첫 프레임을 그대로 반환
    return target0, target1


def cleanup(can0, can1, motors0, motors1):
    for m in motors0 + motors1:
        if not DRY_RUN:
            try:
                m.disable_motor()
            except Exception:
                pass
            try:
                m.disconnect()
            except Exception:
                pass
    if can0:
        can0.stop()
    if can1:
        can1.stop()

# ---------- 세션 실행 ----------
def run_once():
    # CAN 오픈
    can0 = CanWrapper(channel='can0', bitrate=1000000) if ENABLE_CAN0 else None
    can1 = CanWrapper(channel='can1', bitrate=1000000) if ENABLE_CAN1 else None

    if can0 and not can0.start():
        print("CAN0 open failed!"); can0 = None
    if can1 and not can1.start():
        print("CAN1 open failed!"); can1 = None
    if not can0 and not can1:
        print("No CAN bus active. Set ENABLE_CAN0/ENABLE_CAN1.")
        return "exit"

    # 모터 객체
    motors0 = [RobstrideMotor(can0.bus, motor_id=m) for m in MOTOR_IDS_CAN0] if can0 else []
    motors1 = [RobstrideMotor(can1.bus, motor_id=m) for m in MOTOR_IDS_CAN1] if can1 else []

    if DRY_RUN:
        print(f"[DRY-RUN] can0:{bool(can0)} motors0:{[m for m in MOTOR_IDS_CAN0 if can0]} | "
              f"can1:{bool(can1)} motors1:{[m for m in MOTOR_IDS_CAN1 if can1]}")

    # 연결 & 모드
    for m in motors0 + motors1:
        if not DRY_RUN:
            m.connect()
    for m in motors0 + motors1:
        if not DRY_RUN:
            m.set_run_mode(RobstrideMotor.MODE_MIT)
        time.sleep(0.01)

    # 현재 각도 읽기 (부분 실패 시 0.0으로 폴백)
    theta0, theta1 = [], []
    for m in motors0:
        try:
            theta0.append(get_current_angle(can0, m.motor_id))
        except Exception as e:
            print(f"[WARN] id={m.motor_id} 각도 취득 실패: {e} → 0.0 대체")
            theta0.append(0.0)
        time.sleep(0.01)
    for m in motors1:
        try:
            theta1.append(get_current_angle(can1, m.motor_id))
        except Exception as e:
            print(f"[WARN] id={m.motor_id} 각도 취득 실패: {e} → 0.0 대체")
            theta1.append(0.0)
        time.sleep(0.01)

    # Enable
    for m in motors0 + motors1:
        if not DRY_RUN:
            m.enable_motor()
    time.sleep(0.5)

    try:
        # ★ 1) 시작 전 소프트 램프-인 + 타깃 포즈 받기
        try:
            hold0, hold1 = run_startup_ramp(motors0, motors1, theta0, theta1)
        except Exception as e:
            print(f"[WARN] startup ramp 실패: {e}")
            # fallback: 그냥 현재 theta로 유지
            hold0, hold1 = theta0[:], theta1[:]

        # ★ 2) 램프에서 세운 자세를 유지한 채로 space 대기
        #    (이 안에서 계속 명령을 보내서 5,6번 무릎 힘 안 빠지게 함)
        if not hold_posture_until_space(motors0, motors1, hold0, hold1):
            return "restart"  # q 눌렀을 때나 abort 시

        # ★ 3) 트래젝토리 첫 프레임을 현재 hold자세에 맞춰 동기화
        #     -> 갑자기 다른 포즈로 점프하지 않게 해서 풀림 방지
        if positions_list_0:
            # 오른쪽 bus의 포즈 프레임 0을 hold0로 덮어씀
            # (길이 맞춰서 자르거나 늘려주기)
            first0 = positions_list_0[0]
            for i in range(min(len(first0), len(hold0))):
                first0[i] = hold0[i]
        if positions_list_1:
            first1 = positions_list_1[0]
            for i in range(min(len(first1), len(hold1))):
                first1[i] = hold1[i]
        if velocities_list_0:
            velocities_list_0[0] = [0.0]*len(velocities_list_0[0])
        if velocities_list_1:
            velocities_list_1[0] = [0.0]*len(velocities_list_1[0])

        # 트래젝토리 실행
        total_cols0 = len(positions_list_0[0]) if positions_list_0 else 0
        total_cols1 = len(positions_list_1[0]) if positions_list_1 else 0
        print(f"Starting MIT trajectory... (can0:{MOTOR_IDS_CAN0}, cols0={total_cols0} | can1:{MOTOR_IDS_CAN1}, cols1={total_cols1})")
        steps = max(len(positions_list_0), len(positions_list_1))

        for step in range(steps):
            pos_row0 = positions_list_0[step]  if step < len(positions_list_0)  else []
            vel_row0 = velocities_list_0[step] if step < len(velocities_list_0) else []
            pos_row1 = positions_list_1[step]  if step < len(positions_list_1)  else []
            vel_row1 = velocities_list_1[step] if step < len(velocities_list_1) else []

            # can0 (오른쪽: 2,4,6,8)
            for j, m in enumerate(motors0):
                pos = clamp(safe_get(pos_row0, j, 0.0), P_MIN, P_MAX)
                vel = clamp(safe_get(vel_row0, j, 0.0), V_MIN, V_MAX)
                kp_cmd = get_kp_for_id(m.motor_id, KP)
                kd_cmd = get_kd_for_id(m.motor_id, KD)
                torque_cmd = get_torque_for_id(m.motor_id, TORQUE)
                if not DRY_RUN:
                    m.send_operation_mode(pos=pos, vel=vel,
                                          kp=kp_cmd, kd=kd_cmd,
                                          torque=torque_cmd)

            # can1 (왼쪽: 1,3,5,7)
            for j, m in enumerate(motors1):
                pos = clamp(safe_get(pos_row1, j, 0.0), P_MIN, P_MAX)
                vel = clamp(safe_get(vel_row1, j, 0.0), V_MIN, V_MAX)
                kp_cmd = get_kp_for_id(m.motor_id, KP)
                kd_cmd = get_kd_for_id(m.motor_id, KD)
                torque_cmd = get_torque_for_id(m.motor_id, TORQUE)
                if not DRY_RUN:
                    m.send_operation_mode(pos=pos, vel=vel,
                                          kp=kp_cmd, kd=kd_cmd,
                                          torque=torque_cmd)

            time.sleep(dt)

        return "restart"  # 실행 종료 후 사용자 선택 위해 finally로
    except Exception as e:
        print(f"[ERROR] run_once loop: {e}")
        return "restart"
    finally:
        # 종료 복귀: 바로 선택 받고 정리 (shutdown ramp 생략)
        choice = wait_for_choice("스페이스=종료, r=재시작")
        cleanup(can0, can1, motors0, motors1)   # restart여도 항상 정리
        return choice

# ---------- 엔트리포인트 ----------
def main():
    while True:
        choice = run_once()   # "exit" or "restart"
        if choice == "exit":
            break
        # "restart"면 다음 루프에서 새 세션으로 재시작

if __name__ == "__main__":
    main()
