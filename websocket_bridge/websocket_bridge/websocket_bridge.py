#!/usr/bin/env python3
"""
ROS 2 node that

1. Subscribes to:
   - sensor_msgs/msg/PointCloud2
   - sensor_msgs/msg/NavSatFix

2. Streams (or converts) those messages over a WebSocket.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import PointCloud2, NavSatFix

import roslibpy

class WebsocketBridgeNode(Node):

    def __init__(self):
        super().__init__('websocket_pointcloud_bridge')

        # ── Declare parameters with defaults (also match YAML) ──────────────
        self.declare_parameter('pointcloud_topic', '/points_raw')
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('ws_ip', '127.0.0.1')
        self.declare_parameter('ws_port', 9090)
        self.declare_parameter('ws_api_path', '/')
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('send_raw_binary', False)

        # ── Read parameters ──────────────────────────────────────────────────
        pc_topic        = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        gps_topic       = self.get_parameter('gps_topic').get_parameter_value().string_value
        ws_ip           = self.get_parameter('ws_ip').get_parameter_value().string_value
        ws_port         = self.get_parameter('ws_port').get_parameter_value().integer_value
        ws_api_path     = self.get_parameter('ws_api_path').get_parameter_value().string_value
        depth           = self.get_parameter('qos_depth').get_parameter_value().integer_value
        self.raw_binary = self.get_parameter('send_raw_binary').get_parameter_value().bool_value

        qos = QoSProfile(depth=depth)

        # ── Initialise WebSocket connection (TODO) ─────────────────────────
        self.ws_url = f"ws://{ws_ip}:{ws_port}{ws_api_path}"
        self.init_websocket()

        # ── Create subscribers ─────────────────────────────────────────────
        self.create_subscription(PointCloud2, pc_topic, self.pc_callback, qos)
        self.create_subscription(NavSatFix, gps_topic, self.gps_callback, qos)

        self.get_logger().info(
            f"Bridge started → {self.ws_url}\n"
            f" Subscribed to: {pc_topic} (PointCloud2), {gps_topic} (NavSatFix)"
        )

    # ======================================================================
    #                      WebSocket helper stubs
    # ======================================================================

    def init_websocket(self):
        """Connect to WebSocket server. Fill in with chosen client."""
        # TODO: swap these comments for real client initialisation
        pass

    def send_message(self, channel: str, payload):
        """Serialise and send payload over the socket."""
        # TODO: implement encoding – JSON, protobuf, raw bytes, etc.
        pass

    # ======================================================================
    #                             Callbacks
    # ======================================================================

    def pc_callback(self, msg: PointCloud2):
        """Handle incoming PointCloud2."""
        # TODO: choose a sensible serialisation (e.g. convert to Base64)
        self.send_message('pointcloud', data)
        # DEBUG log every N messages if desired

    def gps_callback(self, msg: NavSatFix):
        """Handle incoming NavSatFix."""
        # TODO:
        self.send_message('gps', data)

    # ======================================================================

def main():
    rclpy.init()
    node = WebsocketBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # TODO: close socket gracefully
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
