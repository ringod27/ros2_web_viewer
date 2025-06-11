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
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, NavSatFix, PointField
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from GNSSConverter import GNSSConverter

import roslibpy

from serialization import ros2dict
from websocket.handler import WSHandler, ws_manager

import asyncio
import json
import tornado.web
import tornado.ioloop
import tornado.websocket
import threading
from pyproj import CRS, Transformer
import numpy as np
import struct


class WebsocketBridgeNode(Node):

    def __init__(self):
        super().__init__('websocket_pointcloud_bridge')

        # ── Declare parameters with defaults (also match YAML) ──────────────
        self.declare_parameter('pointcloud_topic', '/sensing/lidar/hesai/pandar')
        self.declare_parameter('gps_topic', '/ublox_gps_node/fix')
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

        # qos = QoSProfile(depth=depth)
        self.qos_profile = QoSProfile(depth=depth)

        # ── Initialise TF buffer and listener ────────────────────────────────
        # TODO listen only when subscribe
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Initialise WebSocket connection (TODO) ─────────────────────────
        # self.ws_url = f"ws://{ws_ip}:{ws_port}{ws_api_path}"
        # self.init_websocket()
        self.init_websocket(ws_ip, ws_port, ws_api_path)

        self._active_subs = {}  # Dictionary to keep track of subscriptions

        self.TYPE_MAP = {
            'pointcloud': (PointCloud2, 'pc_callback'),
            'navsatfix': (NavSatFix, 'gps_callback')
        }

        # ── Create subscribers ─────────────────────────────────────────────
        # self.create_subscription(PointCloud2, pc_topic, self.pc_callback, qos)
        # self.create_subscription(NavSatFix, gps_topic, self.gps_callback, qos)

        # self.get_logger().info(
        #     f"Bridge started → {self.ws_url}\n"
        #     f" Subscribed to: {pc_topic} (PointCloud2), {gps_topic} (NavSatFix)"
        # )

        # GNSSConverter will be created on first GPS fix:
        self.gnss_converter = None

    # ======================================================================
    #                      WebSocket helper stubs
    # ======================================================================
    def init_websocket(self, ip, port, path):
        """Connect to WebSocket server. Fill in with chosen client."""
        # TODO: swap these comments for real client initialisation
        # pass
        def run_server():
            app = tornado.web.Application([
                (path, WSHandler, dict(ros_node=self)),
            ])
            app.listen(port, address=ip)
            print(f"[WS] Server started at ws://{ip}:{port}{path}")
            tornado.ioloop.IOLoop.current().start()

        thread = threading.Thread(target=run_server, daemon=True)
        thread.start()

    def subscribe_to_topic(self, topic_name, topic_type):
        msg_type, callback_name = self.TYPE_MAP.get(topic_type, (None, None))

        if msg_type is None:
            self.get_logger().error(f"Unsupported topic type: {topic_type}")
            return

        callback = getattr(self, callback_name)

        if topic_name in self._active_subs:
            self.get_logger().info(f"Already subscribed to {topic_name}")
            return

        subscription = self.create_subscription(
            msg_type,
            topic_name,
            callback,
            self.qos_profile
        )
        self._active_subs[topic_name] = subscription
        self.get_logger().info(f"Subscribed dynamically to topic: {topic_name}")

    def unsubscribe_from_topic(self, topic_name):
        subscription = self._active_subs.get(topic_name)
        if subscription:
            self.destroy_subscription(subscription)
            del self._active_subs[topic_name]
            self.get_logger().info(f"Unsubscribed from topic: {topic_name}")
        else:
            self.get_logger().warn(f"No active subscription found for: {topic_name}")

    def send_message(self, channel: str, payload):
        """Serialise and send payload over the socket."""
        # TODO: implement encoding – JSON, protobuf, raw bytes, etc.
        # pass
        ws_manager.broadcast(channel, payload)

    # ======================================================================
    #                             Callbacks
    # ======================================================================

    def pc_callback(self, msg: PointCloud2):
        """Handle incoming PointCloud2."""
        # TODO: choose a sensible serialisation (e.g. convert to Base64)
        ros_msg_dict = ros2dict(msg)
        ros_msg_dict["topic"] = "/sensing/lidar/hesai/pandar"
        ros_msg_dict["type"] = "pointcloud"

        self.send_message('pointcloud', ros_msg_dict)

    def gps_callback(self, msg: NavSatFix):
        """Handle incoming NavSatFix."""
        # TODO:
        if self.gnss_converter is None:
            self.gnss_converter = GNSSConverter(
                origin_lat=msg.latitude,
                origin_lon=msg.longitude,
                origin_alt=msg.altitude
            )
            self.get_logger().info(
                f"GNSS origin set to: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}"
            )

        x, y, z = self.gnss_converter.to_local_enu(msg.latitude, msg.longitude, msg.altitude)

        point_in_gnss = PointStamped()
        point_in_gnss.header = msg.header
        point_in_gnss.point.x = x
        point_in_gnss.point.y = y
        point_in_gnss.point.z = z

        try:
            # 2) Lookup transform from gnss_link to 3d_lidar_base_link
            transform = self.tf_buffer.lookup_transform(
                '3d_lidar',  # target frame
                'gnss_link',           # source frame
                rclpy.time.Time())
            
            # 3) Transform the Cartesian point to lidar frame
            point_in_lidar = do_transform_point(point_in_gnss, transform)

            # 4) Send transformed point as dict (or your preferred format)
            payload = {
                'header': {
                    'stamp_sec': point_in_lidar.header.stamp.sec,
                    'stamp_nanosec': point_in_lidar.header.stamp.nanosec,
                    'frame_id': point_in_lidar.header.frame_id,
                },
                'point': {
                    'x': point_in_lidar.point.x,
                    'y': point_in_lidar.point.y,
                    'z': point_in_lidar.point.z,
                }
            }

            payload["topic"] = "/ublox_gps_node/fix"
            payload["type"] = "navsatfix"

            self.send_message('navsatfix', payload)

        except Exception as e:
            self.get_logger().warn(f"Failed to transform GPS point: {e}")
            pass

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
