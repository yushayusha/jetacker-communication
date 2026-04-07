#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct
import time

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range

PORT = '/dev/ttyAMA2'
BAUD = 115200

MSP2_MSG_FLOW = 0x1F02
MSP2_MSG_RANG = 0x1F01


# ── MSP v2 CRC ─────────────────────────────────────────────
def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


# ── MSP v2 parser ──────────────────────────────────────────
class MSP2Parser:
    S_IDLE, S_X, S_DIR, S_FLAGS, S_ID0, S_ID1, S_SZ0, S_SZ1, S_PAYLOAD, S_CRC = range(10)

    def __init__(self):
        self._s = self.S_IDLE
        self._dir = self._flags = self._msg_id = self._size = 0
        self._payload = bytearray()

    def feed(self, chunk: bytes):
        for b in chunk:
            yield from self._byte(b)

    def _byte(self, b):
        s = self._s
        if   s == self.S_IDLE:    self._s = self.S_X if b == 0x24 else self.S_IDLE
        elif s == self.S_X:       self._s = self.S_DIR if b == 0x58 else self.S_IDLE
        elif s == self.S_DIR:     self._dir = b; self._s = self.S_FLAGS
        elif s == self.S_FLAGS:   self._flags = b; self._s = self.S_ID0
        elif s == self.S_ID0:     self._msg_id = b; self._s = self.S_ID1
        elif s == self.S_ID1:     self._msg_id |= b << 8; self._s = self.S_SZ0
        elif s == self.S_SZ0:     self._size = b; self._s = self.S_SZ1
        elif s == self.S_SZ1:
            self._size |= b << 8
            self._payload = bytearray()
            self._s = self.S_PAYLOAD if self._size > 0 else self.S_CRC
        elif s == self.S_PAYLOAD:
            self._payload.append(b)
            if len(self._payload) == self._size:
                self._s = self.S_CRC
        elif s == self.S_CRC:
            body = bytes([
                self._dir, self._flags,
                self._msg_id & 0xFF, (self._msg_id >> 8) & 0xFF,
                self._size & 0xFF, (self._size >> 8) & 0xFF
            ]) + bytes(self._payload)

            self._s = self.S_IDLE
            if crc8_dvb_s2(body) == b:
                yield (self._msg_id, bytes(self._payload))


def decode_flow(payload: bytes):
    if len(payload) < 5:
        return None
    fx = struct.unpack_from('<h', payload, 0)[0]
    fy = struct.unpack_from('<h', payload, 2)[0]
    q  = payload[4]
    return fx, fy, q


def decode_range(payload: bytes):
    if len(payload) < 5:
        return None
    dist_mm = struct.unpack_from('<I', payload, 0)[0]
    q       = payload[4]
    return dist_mm, q


# ── ROS2 Node ──────────────────────────────────────────────
class MTF02PNode(Node):
    def __init__(self):
        super().__init__('mtf02p_node')

        self.declare_parameter('port', PORT)
        port = self.get_parameter('port').value

        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self.parser = MSP2Parser()

        self.flow_pub = self.create_publisher(TwistStamped, '/optical_flow', 10)
        self.range_pub = self.create_publisher(Range, '/range', 10)

        self.last_flow = None
        self.last_range = None

        self.timer = self.create_timer(0.01, self.loop)

        self.get_logger().info(f'MTF-02P node started on {port}')

    def loop(self):
        chunk = self.ser.read(256)
        if not chunk:
            return

        for msg_id, payload in self.parser.feed(chunk):
            if msg_id == MSP2_MSG_FLOW:
                r = decode_flow(payload)
                if r:
                    self.last_flow = r

            elif msg_id == MSP2_MSG_RANG:
                r = decode_range(payload)
                if r:
                    self.last_range = r

        if self.last_flow and self.last_range:
            fx, fy, fq = self.last_flow
            dist_mm, rq = self.last_range

            now = self.get_clock().now().to_msg()

            # ── Publish flow ─────────────────────
            flow_msg = TwistStamped()
            flow_msg.header.stamp = now
            flow_msg.header.frame_id = "optical_flow"

            # approximate scaling (same as your script)
            flow_msg.twist.linear.x = fx / 100.0
            flow_msg.twist.linear.y = fy / 100.0

            self.flow_pub.publish(flow_msg)

            # ── Publish range ────────────────────
            range_msg = Range()
            range_msg.header.stamp = now
            range_msg.header.frame_id = "rangefinder"

            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = 0.1
            range_msg.min_range = 0.01
            range_msg.max_range = 8.0
            range_msg.range = dist_mm / 1000.0

            self.range_pub.publish(range_msg)

            # reset
            self.last_flow = None
            self.last_range = None


# ── main ───────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MTF02PNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()