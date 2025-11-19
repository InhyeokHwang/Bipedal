import struct
import time
import math
import can

class RobstrideMotor:
    P_MIN, P_MAX = -12.57, 12.57
    V_MIN, V_MAX = -44.0, 44.0
    T_MIN, T_MAX = -17.0, 17.0
    KP_MIN, KP_MAX = 0.0, 500.0
    KD_MIN, KD_MAX = 0.0, 5.0

    CMD_TYPE_OPERATION_CONTROL = 0x01
    CMD_TYPE_ENABLE = 0x03
    CMD_TYPE_DISABLE = 0x04
    CMD_TYPE_SET_PARAM = 0x12

    MODE_MIT = 0
    MODE_PP = 1
    MODE_VELOCITY = 2
    MODE_CURRENT = 3
    MODE_CSP = 5

    def __init__(self, can_bus, motor_id=127):
        self.bus = can_bus  # python-can Bus 객체
        self.motor_id = motor_id
        self.master_id = 0xFD
        self.current_mode = None

    def connect(self):
        print("CAN bus is assumed to be ready (socketcan).")
        return True

    def disconnect(self):
        print("Disconnect called (no action for socketcan).")
        return

    def _clip(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)

    def _float_to_uint16(self, x, x_min, x_max):
        clipped_x = self._clip(x, x_min, x_max)
        if x_max == x_min: return 0
        return int(round(((clipped_x - x_min) / (x_max - x_min)) * 65535.0))

    def _float_to_uint12(self, x, x_min, x_max):
        clipped_x = self._clip(x, x_min, x_max)
        if x_max == x_min: return 0
        return int(round(((clipped_x - x_min) / (x_max - x_min)) * 4095.0))

    def _uint16_to_float(self, x, x_min, x_max):
        if x_max == x_min: return x_min
        return (x / 65535.0) * (x_max - x_min) + x_min

    def _send_and_receive(self, can_id, data, is_extended_id=True, timeout=0.0, info=None):
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=is_extended_id)
        # print(f"[TX] CAN ID: {hex(can_id)}, DATA: {data.hex()} | {info if info else ''}")
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"CAN Send Error: {e}")
            return None

        if timeout and timeout > 0:
            t_start = time.time()
            while time.time() - t_start < timeout:
                rx = self.bus.recv(timeout=0.05)
                if rx is not None and rx.arbitration_id != can_id:
                    print(f"[RX] {rx}")
                    return rx
            print("No response received.")
            return None
        # 응답 대기 안 함
        return None

    # ---- Control Functions ----

    def send_operation_mode(self, pos, vel, kp, kd, torque):
        p_int = self._float_to_uint16(pos, self.P_MIN, self.P_MAX)
        v_int = self._float_to_uint16(vel, self.V_MIN, self.V_MAX)
        kp_int = self._float_to_uint16(kp, self.KP_MIN, self.KP_MAX)
        kd_int = self._float_to_uint16(kd, self.KD_MIN, self.KD_MAX)
        data_payload = struct.pack('>HHHH', p_int, v_int, kp_int, kd_int)
        t_int = self._float_to_uint16(torque, self.T_MIN, self.T_MAX)
        can_id = (self.CMD_TYPE_OPERATION_CONTROL << 24) | (t_int << 8) | self.motor_id
        info = {'pos': pos, 'vel': vel, 'kp': kp, 'kd': kd, 'torque': torque}
        return self._send_and_receive(can_id=can_id, data=data_payload, info=info)

    def set_run_mode(self, mode):
        print(f"[Motor {self.motor_id}] Setting run mode to {mode}...")
        data = struct.pack('<H', 0x7005) + b'\x00\x00' + bytes([mode]) + b'\x00\x00\x00'
        can_id = (self.CMD_TYPE_SET_PARAM << 24) | (self.master_id << 8) | self.motor_id
        self.current_mode = mode
        return self._send_and_receive(can_id=can_id, data=data)

    def enable_motor(self):
        print(f"[Motor {self.motor_id}] Enabling motor...")
        can_id = (self.CMD_TYPE_ENABLE << 24) | (self.master_id << 8) | self.motor_id
        return self._send_and_receive(can_id=can_id, data=b'\x00' * 8)

    def disable_motor(self):
        print(f"[Motor {self.motor_id}] Disabling motor...")
        can_id = (self.CMD_TYPE_DISABLE << 24) | (self.master_id << 8) | self.motor_id
        return self._send_and_receive(can_id=can_id, data=b'\x00' * 8)

    def _set_float_parameter(self, name, index, value):
        print(f"[Motor {self.motor_id}] Setting {name} to {value:.2f}...")
        data = struct.pack('<H', index) + b'\x00\x00' + struct.pack('<f', value)
        can_id = (self.CMD_TYPE_SET_PARAM << 24) | (self.master_id << 8) | self.motor_id
        info = {'param': name, 'value': value}
        return self._send_and_receive(can_id=can_id, data=data, info=info)

    def set_position(self, position):
        return self._set_float_parameter("Position", 0x7016, position)

    def set_velocity(self, velocity):
        return self._set_float_parameter("Velocity", 0x700A, velocity)

    def set_current_limit(self, limit):
        return self._set_float_parameter("Current Limit", 0x7018, limit)

    def set_velocity_limit(self, limit):
        return self._set_float_parameter("Velocity Limit", 0x7024, limit)

    def set_acceleration(self, accel):
        return self._set_float_parameter("Acceleration", 0x7025, accel)

    def set_torque_limit(self, torque):
        return self._set_float_parameter("Torque Limit", 0x700B, torque)

    def set_kp(self, kp):
        return self._set_float_parameter("Kp", 0x7010, kp)

    def set_kd(self, kd):
        return self._set_float_parameter("Kd", 0x7011, kd)

    def set_speed_limit(self, limit):
        return self._set_float_parameter("Speed Limit", 0x7017, limit)

    def set_iq_ref(self, current):
        return self._set_float_parameter("Iq Ref", 0x7006, current)

    def get_device_id(self):
        can_id = (0x00 << 24) | (self.master_id << 8) | self.motor_id
        return self._send_and_receive(can_id=can_id, data=b"\x00" * 8)

