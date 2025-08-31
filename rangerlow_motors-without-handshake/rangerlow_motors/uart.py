import serial
import logging
from typing import Union

class UART:
    _baudrate = 115200
    _timeout = 0 # 🔧 неблокирующий режим
    _PACKET_SIZE = 8
    _START_BYTE = 0xA5

    def __init__(self, port: str) -> None:
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=self._baudrate,
                timeout=self._timeout
            )
            self.tmp_buffer = bytearray()
            logging.info(f"UART initialized on {port}")
        except serial.SerialException as e:
            logging.error(f"Failed to open UART port {port}: {e}")
            raise

    @staticmethod
    def calc_sum(packet: Union[bytes, bytearray]) -> int:
        checksum = 0
        for b in packet[:-1]:
            checksum ^= b
        return checksum

    @staticmethod
    def verify_sum(packet: bytearray) -> bool:
        return UART.calc_sum(packet) == packet[-1]

    def read_serial(self) -> Union[bytearray, None]:
        if not self.ser.is_open:
            logging.error('UART port is closed')
            return None

        if self.ser.in_waiting > 0:
            self.tmp_buffer.extend(self.ser.read(self.ser.in_waiting))

        # Sync to start byte
        while len(self.tmp_buffer) >= self._PACKET_SIZE:
            if self.tmp_buffer[0] != self._START_BYTE:
                logging.warning(f"Dropping byte: {self.tmp_buffer[0]}")
                self.tmp_buffer.pop(0)
                continue

            raw_packet = self.tmp_buffer[:self._PACKET_SIZE]

            # Sanity check
            if len(raw_packet) < self._PACKET_SIZE:
                break

            # Slice off the processed packet
            self.tmp_buffer = self.tmp_buffer[self._PACKET_SIZE:]

            if not self.verify_sum(raw_packet):
                logging.warning(f"Invalid checksum for packet: {raw_packet.hex()}")
                continue

            return raw_packet

        return None

    def send_serial(self, raw_bytes: bytes) -> None:
        if self.ser.is_open:
            try:
                self.ser.write(raw_bytes)
                logging.debug(f"Sent: {raw_bytes.hex()}")
            except serial.SerialTimeoutException:
                logging.error("UART transmit timeout")
        else:
            logging.error('UART port is not open')