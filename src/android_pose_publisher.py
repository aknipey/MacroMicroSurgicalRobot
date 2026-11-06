#!/usr/bin/env python3
import rclpy
import websocket
import json
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class SensorDataClient():
    def __init__(self, node, url):
        self.node = node
        self.url = url
        self.publisher = node.create_publisher(PoseStamped, 'android_pose', 10)
        self.test = node.create_publisher(String, 'test', 10)

    def on_message(self, ws, message):
        message_dict = json.loads(message)
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.pose.orientation.w = message_dict['values'][3]
        pose_stamped.pose.orientation.x = message_dict['values'][0]
        pose_stamped.pose.orientation.y = message_dict['values'][1]
        pose_stamped.pose.orientation.z = message_dict['values'][2]
        orientation = pose_stamped.pose.orientation
        self.node.get_logger().info(
            f"Publishing pose: w: {orientation.w}, x: {orientation.x}, y: {orientation.y}, z: {orientation.z}")
        self.publisher.publish(pose_stamped)
        test_msg = String()
        test_msg.data = "test"
        self.test.publish(test_msg)

    def on_error(self, ws, error):
        self.node.get_logger().info("Error, failed to connect to websocket")

    def on_close(self, ws, close_code, reason):
        self.node.get_logger().info("Websocket connection closed")
        self.node.get_logger().info(f"Close code: {close_code}")
        self.node.get_logger().info(f"Reason: {reason}")

    def on_open(self, ws):
        self.node.get_logger().info("Websocket connected")

    def connect(self):
        self.node.get_logger().info("Connecting...")
        ws = websocket.WebSocketApp(self.url, on_open=self.on_open, on_message=self.on_message,
                                    on_error=self.on_error, on_close=self.on_close)
        ws.run_forever()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('android')
    data_client = SensorDataClient(node, "ws://Pixel-3-XL:8081/sensor/connect?type=android.sensor.rotation_vector")
    data_client.connect()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
