import json
import tornado.websocket
from .manager import WebSocketManager

ws_manager = WebSocketManager()

class WSHandler(tornado.websocket.WebSocketHandler):
    def initialize(self, ros_node):
        self.ros_node = ros_node

    def open(self):
        ws_manager.register(self)
        print("[WS] Client connected.")

    def on_close(self):
        ws_manager.unregister(self)
        print("[WS] Client disconnected.")

    def on_message(self, message):
        data = json.loads(message)
        action = data.get("action")
        topic_type = data.get("type")
        topic = data.get("topic")

        if action == "subscribe" and topic:
            self.ros_node.subscribe_to_topic(topic, topic_type)
        elif action == "unsubscribe" and topic:
            self.ros_node.unsubscribe_from_topic(topic)


    def check_origin(self, origin):
        return True
