#!/usr/bin/env python3
"""
Send velocity commands from the Jetson to the STM32 over a serial port.

Frame (18 bytes, little-endian) - must match Core/Inc/cmd_link.h:
    0xA5 0x5A | len = 13 | seq (u8) | vx (f32 m/s) | vy (f32 m/s) | wz (f32 rad/s) | crc16 (u16)
    crc16 = CRC-16/CCITT-FALSE over the len byte and the payload.
    Robot frame: x forward, y left, wz counter-clockwise (same as ROS cmd_vel).

The STM32 stops the robot if no valid frame arrives for 200 ms, so frames must be sent
continuously (50-200 Hz), including while the command is zero.

Usage:
    # constant command (default 0 0 0 = link test, robot holds still)
    python3 jetson_cmd_sender.py --port /dev/ttyTHS1 --vx 0.2

    # forward a ROS 2 geometry_msgs/Twist topic
    python3 jetson_cmd_sender.py --port /dev/ttyTHS1 --source ros2 --topic cmd_vel

In your own code, import pack_frame() and write its result to the serial port.

Needs: pip3 install pyserial   (and a sourced ROS 2 environment for --source ros2)
"""
import argparse
import binascii
import struct
import time

BAUD = 921600


def pack_frame(seq, vx, vy, wz):
    """Build one 18-byte command frame. vx, vy in m/s; wz in rad/s."""
    payload = struct.pack("<Bfff", seq & 0xFF, vx, vy, wz)
    body = bytes([len(payload)]) + payload
    crc = binascii.crc_hqx(body, 0xFFFF)
    return b"\xA5\x5A" + body + struct.pack("<H", crc)


def run_const(ser, rate_hz, vx, vy, wz):
    seq = 0
    period = 1.0 / rate_hz
    nxt = time.monotonic()
    while True:
        ser.write(pack_frame(seq, vx, vy, wz))
        seq += 1
        nxt += period
        time.sleep(max(0.0, nxt - time.monotonic()))


def run_ros2(ser, rate_hz, topic):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    class CmdBridge(Node):
        def __init__(self):
            super().__init__("cmd_serial_bridge")
            self.seq = 0
            self.cmd = (0.0, 0.0, 0.0)
            self.last_msg = 0.0
            self.create_subscription(Twist, topic, self.on_twist, 10)
            # Fixed-rate sending keeps the STM32 timeout fed even if the topic publishes irregularly.
            self.create_timer(1.0 / rate_hz, self.on_timer)

        def on_twist(self, msg):
            self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.last_msg = time.monotonic()

        def on_timer(self):
            # If the topic goes quiet, stop sending: the STM32 then times out and brakes.
            if time.monotonic() - self.last_msg > 0.5:
                return
            ser.write(pack_frame(self.seq, *self.cmd))
            self.seq += 1

    rclpy.init()
    rclpy.spin(CmdBridge())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--source", choices=["const", "ros2"], default="const")
    ap.add_argument("--topic", default="cmd_vel")
    ap.add_argument("--rate", type=float, default=100.0, help="frames per second")
    ap.add_argument("--vx", type=float, default=0.0, help="m/s, forward (const mode)")
    ap.add_argument("--vy", type=float, default=0.0, help="m/s, left (const mode)")
    ap.add_argument("--wz", type=float, default=0.0, help="rad/s, counter-clockwise (const mode)")
    args = ap.parse_args()

    import serial
    ser = serial.Serial(args.port, BAUD, timeout=0)
    if args.source == "ros2":
        run_ros2(ser, args.rate, args.topic)
    else:
        run_const(ser, args.rate, args.vx, args.vy, args.wz)


if __name__ == "__main__":
    main()
