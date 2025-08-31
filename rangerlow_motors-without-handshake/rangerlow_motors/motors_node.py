#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import struct

from rangerros_interfaces.msg import MotorDriver

from .uart import UART


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        # === INIT PARAMS ===
        self.declare_parameter('stm_front_port', '/dev/ttyAMA0')
        self.declare_parameter('stm_mid_port', '/dev/ttyAMA2')
        self.declare_parameter('stm_rear_port', '/dev/ttyAMA4')

        front_port = str(self.get_parameter('stm_front_port').value)
        mid_port = str(self.get_parameter('stm_mid_port').value)
        rear_port = str(self.get_parameter('stm_rear_port').value)

        # === OPEN CONNECTIONS ===
        try:
            self.front_connection = UART(front_port)
        except:
            self.front_connection = None
            self.get_logger().info('NOT CONN FRONT')

        try:
            self.mid_connection = UART(mid_port)
        except:

            self.mid_connection = None
            self.get_logger().info('NOT CONN MID')

        try:
            self.rear_connection = UART(rear_port)
        except:
            self.rear_connection = None
            self.get_logger().info('NOT CONN REAR')

        # === DRIVER SUBCRIBER ===

        self.front_pub_ = self.create_publisher(
            MotorDriver,
            '/rangerlow_motors/front_driver_info',
            10
        )
        self.mid_pub_ = self.create_publisher(
            MotorDriver,
            '/rangerlow_motors/mid_driver_info',
            10
        )

        self.rear_pub_ = self.create_publisher(
            MotorDriver,
            '/rangerlow_motors/rear_driver_info',
            10
        )

        

        self.front_timer = self.create_timer(0.05, self.front_driver_callback)
        self.mid_timer = self.create_timer(0.05, self.mid_driver_callback)
        self.rear_timer = self.create_timer(0.05, self.rear_driver_callback)

        self.front_sub_ = self.create_subscription(
            MotorDriver, '/rangerlow_motors/front_driver', self.front_driver_listener, 10)
        self.front_sub_

        self.mid_sub_ = self.create_subscription(
            MotorDriver, '/rangerlow_motors/mid_driver', self.mid_driver_listener, 10)
        self.mid_sub_


        self.rear_sub_ = self.create_subscription(
            MotorDriver, '/rangerlow_motors/rear_driver', self.rear_driver_listener, 10)
        self.rear_sub_

    def msg_motor_packager(self, core_id, cmd, data):
        start_byte = 0xA5

        data_bytes = struct.pack('<f', data)

        packet = bytearray([start_byte, core_id, cmd])
        packet.extend(data_bytes)

        checksum = 0
        for byte in packet:
            checksum ^= byte

        packet.append(checksum)

        return packet


    def package_to_msg(self, raw_packet):

        msg = MotorDriver()
        if isinstance(raw_packet, bytearray) and len(raw_packet) == 8:
            try:
                rx_buffer = struct.unpack('<BBBiB', raw_packet)

                cmd = rx_buffer[2]
                val = float(rx_buffer[3])
                self.last_core_id = rx_buffer[1]

                if cmd == 26:
                    self.last_m1 = val
                elif cmd == 27:
                    self.last_m2 = val
                    # Публикуем ВСЕГДА оба значения
                    msg.core_id = self.last_core_id
                    msg.m1_cmd = 26
                    msg.m2_cmd = 27
                    msg.m1_data = self.last_m1
                    msg.m2_data = self.last_m2
            except Exception as e:
                self.get_logger().error(f'UNPACK FAILED: {e}')
        return msg


    def front_driver_callback(self):
        if isinstance(self.front_connection, UART):
            raw_packet = self.front_connection.read_serial()

            if isinstance(raw_packet, bytearray) and len(raw_packet) == 8:
                try:
                    rx_buffer = struct.unpack('<BBBiB', raw_packet)
                    cmd = rx_buffer[2]
                    val = float(rx_buffer[3])
                    self.last_core_id = rx_buffer[1]

                    if cmd == 26:
                        self.last_m1 = val
                    elif cmd == 27:
                        self.last_m2 = val

                    # Публикуем ВСЕГДА оба значения
                    msg = MotorDriver()
                    msg.core_id = self.last_core_id
                    msg.m1_cmd = 26
                    msg.m2_cmd = 27
                    msg.m1_data = self.last_m1
                    msg.m2_data = self.last_m2

                    self.front_pub_.publish(msg)
                except Exception as e:
                    self.get_logger().error(f"UNPACK FAILED: {e}")
        

    def mid_driver_callback(self):
        if isinstance(self.mid_connection, UART):
            raw_packet = self.mid_connection.read_serial()

            if isinstance(raw_packet, bytearray) and len(raw_packet) == 8:
                try:
                    rx_buffer = struct.unpack('<BBBiB', raw_packet)
                    cmd = rx_buffer[2]
                    val = float(rx_buffer[3])
                    self.last_core_id = rx_buffer[1]

                    if cmd == 26:
                        self.last_m1 = val
                    elif cmd == 27:
                        self.last_m2 = val

                    # Публикуем ВСЕГДА оба значения
                    msg = MotorDriver()
                    msg.core_id = self.last_core_id
                    msg.m1_cmd = 26
                    msg.m2_cmd = 27
                    msg.m1_data = self.last_m1
                    msg.m2_data = self.last_m2

                    self.mid_pub_.publish(msg)
                except Exception as e:
                    self.get_logger().error(f"UNPACK FAILED: {e}")
        



    def rear_driver_callback(self):
        if isinstance(self.rear_connection, UART):
            raw_packet = self.rear_connection.read_serial()

            if isinstance(raw_packet, bytearray) and len(raw_packet) == 8:
                try:
                    rx_buffer = struct.unpack('<BBBiB', raw_packet)
                    cmd = rx_buffer[2]
                    val = float(rx_buffer[3])
                    self.last_core_id = rx_buffer[1]

                    if cmd == 26:
                        self.last_m1 = val
                    elif cmd == 27:
                        self.last_m2 = val

                    # Публикуем ВСЕГДА оба значения
                    msg = MotorDriver()
                    msg.core_id = self.last_core_id
                    msg.m1_cmd = 26
                    msg.m2_cmd = 27
                    msg.m1_data = self.last_m1
                    msg.m2_data = self.last_m2

                    self.rear_pub_.publish(msg)
                except Exception as e:
                    self.get_logger().error(f"UNPACK FAILED: {e}")
        



    def front_driver_listener(self, msg: MotorDriver):
        if isinstance(self.front_connection, UART):

            m1 = self.msg_motor_packager(msg.core_id, msg.m1_cmd, msg.m1_data)
            m2 = self.msg_motor_packager(msg.core_id, msg.m2_cmd, msg.m2_data)

            self.front_connection.send_serial(m1)
            self.front_connection.send_serial(m2)


       
    def mid_driver_listener(self, msg: MotorDriver):
         if isinstance(self.mid_connection, UART):

            m1 = self.msg_motor_packager(msg.core_id, msg.m1_cmd, msg.m1_data)
            m2 = self.msg_motor_packager(msg.core_id, msg.m2_cmd, msg.m2_data)

            self.mid_connection.send_serial(m1)
            self.mid_connection.send_serial(m2)

    def rear_driver_listener(self, msg: MotorDriver):
         if isinstance(self.rear_connection, UART):

            m1 = self.msg_motor_packager(msg.core_id, msg.m1_cmd, msg.m1_data)
            m2 = self.msg_motor_packager(msg.core_id, msg.m2_cmd, msg.m2_data)

            self.rear_connection.send_serial(m1)
            self.rear_connection.send_serial(m2)

        
def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        node = MotorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('NODE STOPPED')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
