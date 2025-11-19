#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import csv
import sys
from typing import List, Tuple

### Settings
CSV_PATH = "joint_sample.csv"

DESIRED_RIGHT = [
    "right_motor_right_shoulder-right_motor",       
    "right_shoulder_right_top_leg-right_shoulder",
    "right_top_leg_right_bottom_leg--right_top_leg",
    "right_bottom_leg_right_foot-right_bottom_leg",
]

DESIRED_LEFT = [
    "left_motor_left_shoulder--left_motor",       
    "left_shoulder_left_top_leg-left_shoulder",
    "left_top_leg_left_bottom_leg-left_top_leg",
    "left_bottom_leg_left_foot-left_bottom_leg",
]
###


def _read_csv(path: str) -> Tuple[List[str], List[dict]]:
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        header = reader.fieldnames or []
        rows = list(reader)
    if not header:
        raise ValueError("CSV 헤더를 읽지 못했습니다.")
    if not rows:
        raise ValueError("CSV 데이터가 없습니다.")
    return header, rows


def _extract_matrix(rows: List[dict], cols: List[str]) -> List[List[float]]:
    mat: List[List[float]] = []
    for r in rows:
        row_vals = []
        for c in cols:
            try:
                row_vals.append(float(r.get(c, 0.0)))
            except Exception:
                row_vals.append(0.0)
        mat.append(row_vals)
    return mat


def _format_matrix(mat: List[List[float]]) -> str:
    lines = ["["]
    for row in mat:
        lines.append("  [" + ", ".join(f"{v:.15g}" for v in row) + "],")
    lines.append("]")
    return "\n".join(lines)


def _write_py(filename: str, positions: List[List[float]], velocities: List[List[float]], source_csv: str) -> None:
    with open(filename, "w", encoding="utf-8") as f:
        f.write(f"# Auto-generated from {source_csv}\n")
        f.write(f"positions_list = {_format_matrix(positions)}\n\n")
        f.write(f"velocities_list = {_format_matrix(velocities)}\n")
    print(
        f"✅ {filename} 생성 완료 "
        f"(positions: {len(positions)}x{(len(positions[0]) if positions else 0)}, "
        f"velocities: {len(velocities)}x{(len(velocities[0]) if velocities else 0)})"
    )


def _base(h: str) -> str:
    """'pos:XXX' 또는 'vel:XXX' -> 'XXX'"""
    return h.split(":", 1)[1] if ":" in h else h


def _order_cols(cols: List[str], desired_order: List[str]) -> List[str]:
    """
    cols: CSV 헤더 내 컬럼 이름 리스트(예: 'pos:...' or 'vel:...')
    desired_order: 'pos:/vel:' 제거한 베이스 이름 순서
    """
    cmap = {_base(c): c for c in cols}
    ordered = []
    missing = []
    for name in desired_order:
        if name in cmap:
            ordered.append(cmap[name])
        else:
            missing.append(name)
    if missing:
        print(f"⚠️  CSV에 누락된 열 발견(이 이름과 일치하는 헤더가 없음): {missing}")
    # 여분 컬럼이 있으면 경고
    extras = [c for c in cols if _base(c) not in desired_order]
    if extras:
        print(f"ℹ️  DESIRED_*에 명시되지 않은 추가 열(무시됨): {extras}")
    return ordered


def main():
    # 경로 인자 우선
    csv_path = sys.argv[1] if len(sys.argv) >= 2 else CSV_PATH

    header, rows = _read_csv(csv_path)

    # 분류
    pos_left_all  = [h for h in header if h.startswith("pos:") and ":left_"  in h]
    pos_right_all = [h for h in header if h.startswith("pos:") and ":right_" in h]
    vel_left_all  = [h for h in header if h.startswith("vel:") and ":left_"  in h]
    vel_right_all = [h for h in header if h.startswith("vel:") and ":right_" in h]

    if not (pos_left_all or vel_left_all or pos_right_all or vel_right_all):
        raise ValueError("좌/우 pos/vel 관련 컬럼을 하나도 찾지 못했습니다. 헤더명을 확인하세요.")

    # 원하는 순서로 강제 정렬
    pos_right = _order_cols(pos_right_all, DESIRED_RIGHT)
    vel_right = _order_cols(vel_right_all, DESIRED_RIGHT)
    pos_left  = _order_cols(pos_left_all,  DESIRED_LEFT)
    vel_left  = _order_cols(vel_left_all,  DESIRED_LEFT)

    # 각 측 데이터 추출 및 저장
    if pos_left or vel_left:
        left_pos = _extract_matrix(rows, pos_left) if pos_left else [[] for _ in rows]
        left_vel = _extract_matrix(rows, vel_left) if vel_left else [[] for _ in rows]
        _write_py("values_left.py", left_pos, left_vel, csv_path)
    else:
        print("⚠️ left 측 컬럼이 없어 values_left.py를 생성하지 않습니다.")

    if pos_right or vel_right:
        right_pos = _extract_matrix(rows, pos_right) if pos_right else [[] for _ in rows]
        right_vel = _extract_matrix(rows, vel_right) if vel_right else [[] for _ in rows]
        _write_py("values_right.py", right_pos, right_vel, csv_path)
    else:
        print("⚠️ right 측 컬럼이 없어 values_right.py를 생성하지 않습니다.")

    print("완료.")


if __name__ == "__main__":
    main()
