#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from struct import unpack, pack
from rangerlow_proto.msg import RangerProto


class SlipDriverNode(Node):
    _END = 0xC0
    _ESC = 0xDB
    _ESC_END = 0xDC
    _ESC_ESC = 0xDD


    def __init__(self):
        super().__init__('slip_driver_node')
        # Объявляем параметры
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.01)
        self.declare_parameter('poll_interval', 0.01)

        # Получаем значения параметров с помощью .value
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        poll_interval = self.get_parameter('poll_interval').value

        self.slip_read_pub_ = self.create_publisher(RangerProto, '/slip_read', 10)
        self.slip_write_sub_ = self.create_subscription(RangerProto, '/slip_write',self.packet_subscriber, 10)
        self.slip_write_sub_


        # Инициализируем serial-порт
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
        )
        self.packet = []

        # Создаём таймер для периодического чтения пакетов
        self.timer = self.create_timer(poll_interval, self.packet_publisher)

        self.get_logger().info("SLIP DRIVER INITIALIZATION SUCCESS")

    def read_packet(self):
        in_packet = False
        escaped = False
        self.packet = []
        while True:
            byte = self.ser.read(1)
            if not byte:
                return None
            byte = byte[0]

            if not in_packet:
                if byte == self._END:
                    in_packet = True
                continue

            if escaped:
                if byte == self._ESC_END:
                    self.packet.append(self._END)
                elif byte == self._ESC_ESC:
                    self.packet.append(self._ESC)
                escaped = False
            else:
                if byte == self._END:
                    return bytes(self.packet)
                elif byte == self._ESC:
                    escaped = True
                else:
                    self.packet.append(byte)

    def slip_encode(self, data: bytes) -> bytes:
        encoded = bytearray([self._END])
        for byte in data:
            if byte == self._END:
                encoded.extend([self._ESC, self._ESC_END])
            elif byte == self._ESC:
                encoded.extend([self._ESC, self._ESC_ESC])
            else:
                encoded.append(byte)
        encoded.append(self._END)
        return bytes(encoded)

    def write_packet(self, msg: RangerProto):
        payload = b''.join(pack('<f', val) for val in msg.payload)
        length = len(payload)
        packet = bytes([msg.device_id, msg.instruction, 4, length]) + payload
        slip_data = self.slip_encode(packet)
        print("Sent:", slip_data.hex())
        self.ser.write(slip_data)
        self.ser.flush()


    def packet_publisher(self):
        packet = self.read_packet()
        if packet:
            msg = RangerProto()
            msg.device_id = packet[0]
            msg.instruction = packet[1]
            types = packet[2]
            length = packet[3]
            payload = []
            p = packet[4:4+length]
            for i in range(0, length,4):
                inter = p[i:i+4]
                if len(inter) == 4:
                    val = unpack('<f', bytes(inter))[0]
                    payload.append(val)
            msg.payload = payload
            self.slip_read_pub_.publish(msg)

    def packet_subscriber(self, msg: RangerProto):
        self.write_packet(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SlipDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()