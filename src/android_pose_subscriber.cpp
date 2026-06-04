#include <string>
#include <sstream>
#include <vector>
#include <array>
#include <math.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "omni_msgs/msg/omni_state.hpp"
#include "omni_msgs/msg/omni_button_event.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Touch position coordinates are output in millimetres
const double TOUCH_POSITION_UNIT_SCALE_FACTOR = 1e-3;

const double EPSILON = 1e-6;

bool IsValidDouble(double value)
{
    return !(isnan(value) || abs(value) < EPSILON);
}

bool IsValidQuaternion(tf2::Quaternion quaternion)
{
    return IsValidDouble(quaternion.getX()) &&
           IsValidDouble(quaternion.getY()) &&
           IsValidDouble(quaternion.getZ()) &&
           IsValidDouble(quaternion.getW());
}

tf2::Quaternion QuaternionExponential(tf2::Quaternion quaternion)
{
    tf2::Vector3 vectorComponent(quaternion.getX(), quaternion.getY(), quaternion.getZ());

    double vectorMagnitude = vectorComponent.length();
    double expW = exp(quaternion.getW());

    tf2::Vector3 expVectorComponent = expW * vectorComponent.normalized() * sin(vectorMagnitude);

    tf2::Quaternion result(
        expVectorComponent.getX(),
        expVectorComponent.getY(),
        expVectorComponent.getZ(),
        expW * cos(vectorMagnitude)
    );

    return result;
}

tf2::Quaternion QuaternionLogarithm(tf2::Quaternion quaternion)
{
    tf2::Vector3 vectorComponent(quaternion.getX(), quaternion.getY(), quaternion.getZ());

    double magnitude = quaternion.length();

    tf2::Vector3 logVectorComponent = vectorComponent.normalized() * acos(quaternion.getW() / magnitude);

    tf2::Quaternion result(
        logVectorComponent.getX(),
        logVectorComponent.getY(),
        logVectorComponent.getZ(),
        log(magnitude)
    );

    return result;
}

tf2::Quaternion GetAngularVelocitySimple(
    const geometry_msgs::msg::PoseStamped& prevPose,
    const geometry_msgs::msg::PoseStamped& currentPose,
    const double dt
)
{
    // Time difference between the two pose measurements
    double poseDt = (rclcpp::Time(currentPose.header.stamp) - rclcpp::Time(prevPose.header.stamp)).seconds();

    // Convert orientations from quaternion messages to tf2 quaternions
    tf2::Quaternion prevOrientation, currentOrientation;
    tf2::fromMsg(prevPose.pose.orientation, prevOrientation);
    tf2::fromMsg(currentPose.pose.orientation, currentOrientation);

    tf2::Quaternion diffQuater = currentOrientation * prevOrientation.inverse();

    tf2::Quaternion prevConjQuaternion;

    prevConjQuaternion.setX(-(diffQuater.x()));
    prevConjQuaternion.setY(-(diffQuater.y()));
    prevConjQuaternion.setZ(-(diffQuater.z()));
    prevConjQuaternion.setW(diffQuater.getW());

    tf2::Quaternion output = QuaternionExponential(QuaternionLogarithm(diffQuater) / poseDt) * prevConjQuaternion * 2;

    return output;
}

// Function to calculate the average angular velocity from two timestamped poses and the desired time delta in seconds
tf2::Quaternion GetAngularVelocity(
    const geometry_msgs::msg::PoseStamped& prevPose,
    const geometry_msgs::msg::PoseStamped& currentPose
)
{
    // Time difference between the two pose measurements
    double poseDt = (rclcpp::Time(currentPose.header.stamp) - rclcpp::Time(prevPose.header.stamp)).seconds();

    // Convert orientations from quaternion messages to tf2 quaternions
    tf2::Quaternion prevOrientation, currentOrientation;
    tf2::fromMsg(prevPose.pose.orientation, prevOrientation);
    tf2::fromMsg(currentPose.pose.orientation, currentOrientation);

    tf2::Quaternion prevConjQuaternion = prevOrientation.inverse();

    tf2::Quaternion diffQuater = currentOrientation * prevConjQuaternion;

    tf2::Quaternion output = QuaternionExponential((QuaternionLogarithm(diffQuater) * 2) / poseDt) * prevConjQuaternion;

    return output;
}

void TransformStampedToPoseStamped(const geometry_msgs::msg::TransformStamped& transformStamped, geometry_msgs::msg::PoseStamped& poseStamped)
{
    poseStamped.header = transformStamped.header;

    poseStamped.pose.position.x = transformStamped.transform.translation.x;
    poseStamped.pose.position.y = transformStamped.transform.translation.y;
    poseStamped.pose.position.z = transformStamped.transform.translation.z;

    poseStamped.pose.orientation = transformStamped.transform.rotation;
}

std::string QuaternionToString(geometry_msgs::msg::Quaternion quaternion)
{
    std::stringstream output;

    output << "x: "
        << quaternion.x
        << ", y: "
        << quaternion.y
        << ", z: "
        << quaternion.z
        << ", w: "
        << quaternion.w;

    return output.str();
}

std::string QuaternionToString(tf2::Quaternion quaternion)
{
    std::stringstream output;

    output << "x: "
        << quaternion.getX()
        << ", y: "
        << quaternion.getY()
        << ", z: "
        << quaternion.getZ()
        << ", w: "
        << quaternion.getW();

    return output.str();
}

std::string PoseToString(geometry_msgs::msg::Pose pose, const double position_unit_scale_factor)
{
    std::stringstream output;

    output << "Position: (x: "
        << pose.position.x * position_unit_scale_factor
        << "m, y: "
        << pose.position.y * position_unit_scale_factor
        << "m, z: "
        << pose.position.z * position_unit_scale_factor
        << "m), Orientation: ("
        << QuaternionToString(pose.orientation)
        << ")";

    return output.str();
}

std::string Vector3ToString(geometry_msgs::msg::Vector3 vector3)
{
    std::stringstream output;

    output << "(x: "
        << vector3.x
        << ", y: "
        << vector3.y
        << ", z: "
        << vector3.z
        << ")";

    return output.str();
}

void poseCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& poseStamped,
    geometry_msgs::msg::PoseStamped& currentPoseStamped,
    geometry_msgs::msg::PoseStamped& prevPoseStamped,
    tf2::Quaternion& angularVelocity
)
{
    prevPoseStamped = currentPoseStamped;
    currentPoseStamped = *poseStamped;

    angularVelocity = GetAngularVelocity(prevPoseStamped, currentPoseStamped);
    RCLCPP_INFO(rclcpp::get_logger("android_pose_subscriber"), "Received pose, stamp (%d.%d): %s", poseStamped->header.stamp.sec, poseStamped->header.stamp.nanosec, PoseToString(poseStamped->pose, 1).c_str());
    RCLCPP_INFO(rclcpp::get_logger("android_pose_subscriber"), "Angular velocity: %s", QuaternionToString(angularVelocity).c_str());
}

void touchStateCallback(
    rclcpp::Node::SharedPtr node,
    const omni_msgs::msg::OmniState::ConstSharedPtr& omniState,
    geometry_msgs::msg::PoseStamped& poseStamped,
    geometry_msgs::msg::PoseStamped& prevPoseStamped,
    tf2::Quaternion& angularVelocity
)
{
    prevPoseStamped = geometry_msgs::msg::PoseStamped(poseStamped);
    poseStamped.header.stamp = node->now();
    poseStamped.pose = omniState->pose;

    // Scale touch position units to metres
    poseStamped.pose.position.x = TOUCH_POSITION_UNIT_SCALE_FACTOR * omniState->pose.position.x;
    poseStamped.pose.position.y = TOUCH_POSITION_UNIT_SCALE_FACTOR * omniState->pose.position.y;
    poseStamped.pose.position.z = TOUCH_POSITION_UNIT_SCALE_FACTOR * omniState->pose.position.z;

    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Current Touch Pose: %s, Prev Touch Pose: %s", PoseToString(poseStamped.pose, 1).c_str(), PoseToString(prevPoseStamped.pose, 1).c_str());

    angularVelocity = GetAngularVelocity(prevPoseStamped, poseStamped);

    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Current Touch Angular Velocity: x: %.3f, y: %.3f, z: %.3f, w: %.3f", angularVelocity.getX(), angularVelocity.getY(), angularVelocity.getZ(), angularVelocity.getW());

    geometry_msgs::msg::Vector3 current = omniState->current;

    // RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Touch State:\n\tPose: %s\n\tCurrent: %s\n\tVelocity: %s",
    //     PoseToString(omniState->pose, TOUCH_POSITION_UNIT_SCALE_FACTOR).c_str(),
    //     Vector3ToString(omniState->current).c_str(),
    //     Vector3ToString(omniState->velocity).c_str()
    // );
}

int main(int argc, char **argv)
{
    geometry_msgs::msg::PoseStamped prevPoseStamped;
    geometry_msgs::msg::PoseStamped currentPoseStamped;
    tf2::Quaternion currentAngularVelocity;

    prevPoseStamped.pose.orientation.w = 0.5;
    prevPoseStamped.pose.orientation.x = 0.5;
    prevPoseStamped.pose.orientation.y = 0.5;
    prevPoseStamped.pose.orientation.z = 0.5;

    currentPoseStamped.pose.orientation.w = 0.4;
    currentPoseStamped.pose.orientation.x = 0.4;
    currentPoseStamped.pose.orientation.y = 0.4;
    currentPoseStamped.pose.orientation.z = 0.4;

    currentAngularVelocity.setW(0.5);
    currentAngularVelocity.setX(0.5);
    currentAngularVelocity.setY(0.5);
    currentAngularVelocity.setZ(0.5);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("android_pose_subscriber");

    // auto poseSubscriber = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    //     "android_pose",
    //     rclcpp::QoS(1),
    //     [&](const geometry_msgs::msg::PoseStamped::ConstSharedPtr poseStamped)
    //     {
    //         poseCallback(
    //             poseStamped,
    //             currentPoseStamped,
    //             prevPoseStamped,
    //             currentAngularVelocity
    //         );
    //     }
    // );

    auto touchStateSubscriber = node->create_subscription<omni_msgs::msg::OmniState>(
        "/phantom/state",
        rclcpp::QoS(1),
        [&](const omni_msgs::msg::OmniState::ConstSharedPtr omniState)
        {
            touchStateCallback(
                node,
                omniState,
                currentPoseStamped,
                prevPoseStamped,
                currentAngularVelocity
            );
        }
    );

    auto vis_pub = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", rclcpp::QoS(10));

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = rclcpp::Time(0, 0);
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    vis_pub->publish(marker);

    marker.action = visualization_msgs::msg::Marker::MODIFY;

    rclcpp::Rate rate(100);

    RCLCPP_INFO(node->get_logger(), "Starting subscriber");

    while (rclcpp::ok())
    {
        if (IsValidQuaternion(currentAngularVelocity))
        {
            tf2::Quaternion markerOrientation;
            tf2::fromMsg(marker.pose.orientation, markerOrientation);
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Prev marker orientation: %s", QuaternionToString(marker.pose.orientation).c_str());
            geometry_msgs::msg::Quaternion newOrientation = tf2::toMsg((currentAngularVelocity * 1e-5 * markerOrientation).normalize());
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "New marker orientation: %s", QuaternionToString(newOrientation).c_str());
            marker.pose.orientation = newOrientation;
            vis_pub->publish(marker);

            currentAngularVelocity.setX(0);
            currentAngularVelocity.setY(0);
            currentAngularVelocity.setZ(0);
            currentAngularVelocity.setW(1);
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
