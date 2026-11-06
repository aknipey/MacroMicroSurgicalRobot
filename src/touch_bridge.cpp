// ROS side of the Touch driver: publishes what omni_state publishes (same topics,
// messages, axes and units) from touch_server's UDP state packets, and forwards
// NAME/force_feedback to touch_server. touch_server owns the device, in a process
// without ROS libraries, as OpenHaptics renders no force in a ROS process (see
// src/touch_server.cpp).

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "macro_micro_robot/touch_packets.hpp"
#include "omni_msgs/msg/omni_button_event.hpp"
#include "omni_msgs/msg/omni_feedback.hpp"
#include "omni_msgs/msg/omni_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class TouchBridge : public rclcpp::Node
{
public:
    TouchBridge() : Node("omni_haptic_node")
    {
        const std::string name = declare_parameter<std::string>("omni_name", "phantom");
        reference_frame_ = declare_parameter<std::string>("reference_frame", "/map");
        // QoS as omni_state
        state_pub_ = create_publisher<omni_msgs::msg::OmniState>(name + "/state", 1);
        button_pub_ = create_publisher<omni_msgs::msg::OmniButtonEvent>(name + "/button", 100);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(name + "/pose", 1);
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(name + "/joint_states", 1);
        // Not in omni_state: the force touch_server actually rendered (force_feedback + scene)
        applied_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(name + "/applied_force", 10);
        force_sub_ = create_subscription<omni_msgs::msg::OmniFeedback>(
            name + "/force_feedback", 1, [this](omni_msgs::msg::OmniFeedback::SharedPtr msg) { on_force(*msg); });

        state_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(TOUCH_STATE_PORT);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (bind(state_socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0)
            throw std::runtime_error("cannot listen on udp " + std::to_string(TOUCH_STATE_PORT) +
                                     " - is another touch_bridge running?");
        timeval timeout{0, 200000};
        setsockopt(state_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        force_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        force_address_.sin_family = AF_INET;
        force_address_.sin_port = htons(TOUCH_FORCE_PORT);
        force_address_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

        receiver_ = std::thread([this] { receive(); });
        RCLCPP_INFO(get_logger(), "Touch bridge: %s/state, /button, /pose, /joint_states from touch_server; "
                                  "%s/force_feedback to it", name.c_str(), name.c_str());
    }

    ~TouchBridge() override
    {
        send_force(0.0, 0.0, 0.0);
        running_ = false;
        if (receiver_.joinable())
            receiver_.join();
        close(state_socket_);
        close(force_socket_);
    }

private:
    void on_force(const omni_msgs::msg::OmniFeedback &msg)
    {
        send_force(msg.force.x, msg.force.y, msg.force.z);
    }

    void send_force(double x, double y, double z)
    {
        TouchForcePacket packet{TOUCH_FORCE_MAGIC, 0, {x, y, z}};
        sendto(force_socket_, &packet, sizeof(packet), 0,
               reinterpret_cast<sockaddr *>(&force_address_), sizeof(force_address_));
    }

    void receive()
    {
        bool connected = false;
        while (running_ && rclcpp::ok()) {
            TouchStatePacket packet;
            ssize_t size = recv(state_socket_, &packet, sizeof(packet), 0);
            if (size != static_cast<ssize_t>(sizeof(packet)) || packet.magic != TOUCH_STATE_MAGIC) {
                if (size < 0 && connected) {
                    RCLCPP_WARN(get_logger(), "No data from touch_server - is it running?");
                    connected = false;
                }
                continue;
            }
            if (!connected) {
                RCLCPP_INFO(get_logger(), "Receiving Touch state from touch_server");
                connected = true;
            }
            publish(packet);
        }
    }

    // omni_state's publish_omni_state, from a packet
    void publish(const TouchStatePacket &packet)
    {
        const auto stamp = now();
        omni_msgs::msg::OmniState state;
        state.header.stamp = stamp;
        state.locked = packet.white_button;
        state.close_gripper = packet.grey_button;
        state.pose.position.x = packet.position[0];
        state.pose.position.y = packet.position[1];
        state.pose.position.z = packet.position[2];
        state.pose.orientation.x = packet.orientation[0];
        state.pose.orientation.y = packet.orientation[1];
        state.pose.orientation.z = packet.orientation[2];
        state.pose.orientation.w = packet.orientation[3];
        state.velocity.x = packet.velocity[0];
        state.velocity.y = packet.velocity[1];
        state.velocity.z = packet.velocity[2];
        state_pub_->publish(state);

        sensor_msgs::msg::JointState joints;
        joints.header.stamp = stamp;
        joints.name = {"waist", "shoulder", "elbow", "yaw", "pitch", "roll"};
        joints.position = {-packet.thetas[1], packet.thetas[2], packet.thetas[3],
                           -packet.thetas[4] + M_PI, -packet.thetas[5] - 3 * M_PI / 4, -packet.thetas[6] - M_PI};
        joint_pub_->publish(joints);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = reference_frame_;
        pose.pose = state.pose;
        pose.pose.position.x /= 1000.0;
        pose.pose.position.y /= 1000.0;
        pose.pose.position.z /= 1000.0;
        pose_pub_->publish(pose);

        geometry_msgs::msg::Vector3Stamped applied;
        applied.header.stamp = stamp;
        applied.vector.x = packet.force[0];
        applied.vector.y = packet.force[1];
        applied.vector.z = packet.force[2];
        applied_pub_->publish(applied);

        if (packet.grey_button != grey_ || packet.white_button != white_) {
            omni_msgs::msg::OmniButtonEvent event;
            event.grey_button = packet.grey_button;
            event.white_button = packet.white_button;
            button_pub_->publish(event);
            grey_ = packet.grey_button;
            white_ = packet.white_button;
        }
    }

    std::string reference_frame_;
    rclcpp::Publisher<omni_msgs::msg::OmniState>::SharedPtr state_pub_;
    rclcpp::Publisher<omni_msgs::msg::OmniButtonEvent>::SharedPtr button_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr applied_pub_;
    rclcpp::Subscription<omni_msgs::msg::OmniFeedback>::SharedPtr force_sub_;
    int state_socket_ = -1, force_socket_ = -1;
    sockaddr_in force_address_{};
    std::thread receiver_;
    std::atomic<bool> running_{true};
    int grey_ = 0, white_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TouchBridge>();
    rclcpp::spin(node);
    node.reset();       // sends the zero force
    rclcpp::shutdown();
    return 0;
}
