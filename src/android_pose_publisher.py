#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import websocket
import json
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class SensorDataClient():
    def __init__(this, node, url):
        this.node = node
        this.url = url
        this.publisher = node.create_publisher(PoseStamped, 'android_pose', 10)
        this.test = node.create_publisher(String, 'test', 10)

    def on_message(this, ws, message):
        message_dict = json.loads(message)
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = this.node.get_clock().now().to_msg()
        pose_stamped.pose.orientation.w = message_dict['values'][3]
        pose_stamped.pose.orientation.x = message_dict['values'][0]
        pose_stamped.pose.orientation.y = message_dict['values'][1]
        pose_stamped.pose.orientation.z = message_dict['values'][2]
        this.node.get_logger().info(f"Publishing pose: w: {pose_stamped.pose.orientation.w}, x: {pose_stamped.pose.orientation.x}, y: {pose_stamped.pose.orientation.y}, z: {pose_stamped.pose.orientation.z}")
        this.publisher.publish(pose_stamped)
        test_msg = String()
        test_msg.data = "test"
        this.test.publish(test_msg)

    def on_error(this, ws, error):
        this.node.get_logger().info("Error, failed to connect to websocket")

    def on_close(this, ws, close_code, reason):
        this.node.get_logger().info("Websocket connection closed")
        this.node.get_logger().info(f"Close code: {close_code}")
        this.node.get_logger().info(f"Reason: {reason}")

    def on_open(this, ws):
        this.node.get_logger().info("Websocket connected")

    def connect(this):
        this.node.get_logger().info("Connecting...")
        ws = websocket.WebSocketApp(this.url, on_open=this.on_open, on_message=this.on_message, on_error=this.on_error, on_close=this.on_close)
        ws.run_forever()

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('android')
    dataClient = SensorDataClient(node, "ws://Pixel-3-XL:8081/sensor/connect?type=android.sensor.rotation_vector")
    dataClient.connect()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
