import can
import threading
import queue
import time

class CanWrapper:
    """
    python-can 기반 CAN 송수신 래퍼.
    송신: send_message()
    수신: get_message()
    """
    def __init__(self, channel='can0', bitrate=1000000, interface='socketcan'):
        self.channel = channel
        self.bitrate = bitrate
        self.interface = interface
        self.bus = None
        self.rx_queue = queue.Queue()
        self.running = False
        self.rx_thread = None

    def start(self):
        try:
            self.bus = can.interface.Bus(channel=self.channel, interface=self.interface, bitrate=self.bitrate)
            self.running = True
            self.rx_thread = threading.Thread(target=self._rx_worker)
            self.rx_thread.daemon = True
            self.rx_thread.start()
            print(f"CAN bus opened on {self.channel} ({self.interface}) at {self.bitrate} bps.")
            return True
        except Exception as e:
            print(f"Error opening CAN bus: {e}")
            return False

    def stop(self):
        self.running = False
        if self.rx_thread:
            self.rx_thread.join()
        if self.bus:
            self.bus.shutdown()
            print("CAN communication stopped.")

    def _rx_worker(self):
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.05)
                if msg is not None:
                    self.rx_queue.put(msg)
            except Exception as e:
                print(f"CAN receive error: {e}")
            time.sleep(0.001)

    def send_message(self, arbitration_id, data, is_extended_id=True):
        """
        CAN 프레임 송신
        :param arbitration_id: CAN ID (int)
        :param data: bytes 또는 bytearray(최대 8바이트)
        :param is_extended_id: 확장 프레임 여부
        """
        try:
            msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=is_extended_id)
            self.bus.send(msg)
            # print(f"[TX] CAN ID: {hex(arbitration_id)}, DATA: {data.hex()}")
        except can.CanError as e:
            print(f"CAN Send Error: {e}")

    def get_message(self, timeout=1.0):
        """
        큐에서 CAN 수신 메시지 하나 꺼내기
        :param timeout: 대기 시간 (초)
        :return: python-can Message 객체 or None
        """
        try:
            return self.rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def clear_rx_queue(self):
        with self.rx_queue.mutex:
            self.rx_queue.queue.clear()
