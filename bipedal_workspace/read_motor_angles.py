#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
read_motor_angles.py

SocketCAN(can0)에서 Robostride 모터들의 현재 각도(rad)만 읽어 출력.
- 상태 요청(ID type=0x02, 확장프레임)을 직접 날려 응답을 파싱합니다.
- --once 옵션으로 1회만 출력하거나, 기본은 지정한 주기로 반복 출력합니다.

예)
  python3 read_motor_angles.py --ids 1,2,5 --hz 10
  python3 read_motor_angles.py --ids 5 --once
"""

import sys
import time
import math
import argparse
import can  # python-can
from typing import Dict, Optional, List

# --- 연결 설정 (SocketCAN) ---
DEFAULT_CAN_CHANNEL = "can1"
MASTER_ID = 0x00  # 모터 파라미터 CAN_MASTER(0x200b)와 일치해야 함(일반적으로 0x00)

# --- 상태 변환 상수(펌웨어 스케일) ---
P_MIN, P_MAX = -12.57, 12.57   # ≈ ±4π
V_MIN, V_MAX = -44.0, 44.0
T_MIN, T_MAX = -17.0, 17.0

def _u16_to_float(x: int, x_min: float, x_max: float) -> float:
    return (x / 65535.0) * (x_max - x_min) + x_min

def _read_status_once(bus: can.Bus, motor_id: int, master_id: int = MASTER_ID, timeout: float = 0.2) -> Optional[Dict[str, float]]:
    """
    모터에 상태 프레임(Type=0x02)을 1회 요청하고 응답을 파싱.
    성공 시 dict(angle_rad, speed_rad_s, torque_nm, temp_c), 실패 시 None
    """
    req_id = (0x02 << 24) | (master_id << 8) | motor_id  # 확장 ID
    try:
        bus.send(can.Message(arbitration_id=req_id, data=b'\x00' * 8, is_extended_id=True))
    except Exception as e:
        print(f"[WARN] status 요청 전송 실패(motor {motor_id}): {e}")
        return None

    t_end = time.time() + timeout
    while time.time() < t_end:
        resp = bus.recv(timeout=max(0.0, t_end - time.time()))
        if resp is None:
            break

        rid = resp.arbitration_id
        rtype = (rid >> 24) & 0x3F
        rmotor = (rid >> 8) & 0xFF
        if not resp.is_extended_id or rtype != 0x02 or rmotor != motor_id:
            # 다른 프레임이면 계속
            continue

        data = bytes(resp.data)
        if len(data) < 8:
            continue

        angle_raw  = int.from_bytes(data[0:2], 'big')
        speed_raw  = int.from_bytes(data[2:4], 'big')
        torque_raw = int.from_bytes(data[4:6], 'big')
        temp_raw   = int.from_bytes(data[6:8], 'big')

        return {
            'angle_rad':   _u16_to_float(angle_raw,  P_MIN, P_MAX),
            'speed_rad_s': _u16_to_float(speed_raw,  V_MIN, V_MAX),
            'torque_nm':   _u16_to_float(torque_raw, T_MIN, T_MAX),
            'temp_c':      temp_raw / 10.0,
        }
    return None

def get_angle_rad(bus: can.Bus, motor_id: int, master_id: int = MASTER_ID, timeout: float = 0.2) -> Optional[float]:
    st = _read_status_once(bus, motor_id, master_id, timeout)
    return None if st is None else st['angle_rad']

def parse_ids(ids_str: str) -> List[int]:
    try:
        return [int(x.strip()) for x in ids_str.split(",") if x.strip() != ""]
    except Exception:
        raise argparse.ArgumentTypeError("모터 ID 목록 파싱 실패. 예: --ids 1,2,5")

def main():
    ap = argparse.ArgumentParser(description="Robostride 모터 각도 리더")
    ap.add_argument("--channel", default=DEFAULT_CAN_CHANNEL, help="SocketCAN 채널명 (기본: can0)")
    ap.add_argument("--ids", type=parse_ids, required=True, help="읽을 모터 ID들 (쉼표구분). 예: 1,2,5")
    ap.add_argument("--hz", type=float, default=5.0, help="반복 출력 주기(Hz). --once와 함께 사용하면 무시")
    ap.add_argument("--timeout", type=float, default=0.2, help="각 모터 응답 타임아웃(초)")
    ap.add_argument("--once", action="store_true", help="1회만 읽고 종료")
    ap.add_argument("--master", type=lambda x: int(x, 0), default=MASTER_ID, help="MASTER_ID (기본 0x00, 16진수 허용 ex: 0x00)")
    args = ap.parse_args()

    # SocketCAN 열기
    try:
        bus = can.interface.Bus(channel=args.channel, bustype='socketcan')
    except Exception as e:
        print(f"[ERROR] CAN 버스({args.channel}) 열기 실패: {e}")
        print(" - can0가 UP 상태인지 확인:  sudo ip link set can0 up type can bitrate 1000000")
        print(" - 권한/드라이버(gs_usb 등) 점검, dmesg 확인 필요")
        sys.exit(1)

    ids = args.ids
    period = 1.0 / max(1e-6, args.hz)

    def read_and_print_once():
        ts = time.strftime("%H:%M:%S")
        line_parts = [f"[{ts}]"]
        for mid in ids:
            ang = get_angle_rad(bus, mid, master_id=args.master, timeout=args.timeout)
            if ang is None:
                line_parts.append(f"ID{mid}: N/A")
            else:
                line_parts.append(f"ID{mid}: {ang:+.3f} rad ({math.degrees(ang):+.1f}°)")
        print("  ".join(line_parts))

    try:
        if args.once:
            read_and_print_once()
        else:
            while True:
                t0 = time.time()
                read_and_print_once()
                dt = time.time() - t0
                time.sleep(max(0.0, period - dt))
    except KeyboardInterrupt:
        print("\n[Ctrl+C] 종료")
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
