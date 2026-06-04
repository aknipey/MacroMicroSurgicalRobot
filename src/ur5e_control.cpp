#include <string>
#include <sstream>
#include <vector>
#include <array>
#include <math.h>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "omni_msgs/msg/omni_state.hpp"
#include "omni_msgs/msg/omni_button_event.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "cartesian_control_msgs/action/follow_cartesian_trajectory.hpp"
#include "cartesian_control_msgs/msg/cartesian_trajectory_point.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "../include/Pose.hpp"

using namespace std::chrono_literals;

// Touch position coordinates are output in millimetres, must convert to metres
const double TOUCH_POSITION_UNIT_SCALE_FACTOR = 1e-3;

// Period in seconds of each UR5e pose control operation
const double UR5E_CONTROL_PERIOD = 1e-2;

// Number of subdivisions of a single trajectory to send to the UR5E, increase number to improve motion smoothness
const double UR5E_CONTROL_NUM_SUBDIVISIONS = 10;

// The duration, in seconds, of a single pose trajectory command sent to the UR5e
const double UR5E_CONTROL_TIME_INCREMENT = UR5E_CONTROL_PERIOD / UR5E_CONTROL_NUM_SUBDIVISIONS;

// The maximum age, in seconds, of the latest pose update received from the UR5e. Past this age, control is halted until the next update.
const double UR5E_MAX_POSE_AGE = 0.1;

// Maximum tolerable uncertainty in a double
const double EPSILON = 1e-6;

// Factor by which Touch translational movements are scaled before being sent to the UR5e
const double UR5E_TRANSLATION_SCALE_FACTOR = 3;

// Factor by which Touch rotational movements are scaled before being sent to the UR5e
const double UR5E_ROTATION_SCALE_FACTOR = 0.2;

const std::string UR5E_CONTROLLER_NAME = "pose_based_cartesian_traj_controller";

const std::string UR5E_ACTION_CLIENT_NAME = "pose_based_cartesian_traj_controller/follow_cartesian_trajectory";

using FollowCartesianTrajectory = cartesian_control_msgs::action::FollowCartesianTrajectory;

bool IsZero(double value)
{
    return abs(value) < EPSILON;
}

bool IsValidQuaternion(tf2::Quaternion quaternion)
{
    return !((isnan(quaternion.getX()) || isnan(quaternion.getY()) || isnan(quaternion.getZ()) || isnan(quaternion.getW())) ||
           (IsZero(quaternion.getX()) && IsZero(quaternion.getY()) && IsZero(quaternion.getZ()) && IsZero(quaternion.getW())));
}

void TransformStampedToPoseStamped(const geometry_msgs::msg::TransformStamped& transformStamped, geometry_msgs::msg::PoseStamped& poseStamped)
{
    poseStamped.header = transformStamped.header;

    poseStamped.pose.position.x = transformStamped.transform.translation.x;
    poseStamped.pose.position.y = transformStamped.transform.translation.y;
    poseStamped.pose.position.z = transformStamped.transform.translation.z;

    poseStamped.pose.orientation = transformStamped.transform.rotation;
}

std::string PoseToString(Pose pose, const double position_unit_scale_factor)
{
    std::stringstream output;

    output << "Position: (x: "
        << pose.position.getX() * position_unit_scale_factor
        << "m, y: "
        << pose.position.getY() * position_unit_scale_factor
        << "m, z: "
        << pose.position.getZ() * position_unit_scale_factor
        << "m), Orientation: (x: "
        << pose.orientation.getX()
        << ", y: "
        << pose.orientation.getY()
        << ", z: "
        << pose.orientation.getZ()
        << ", w: "
        << pose.orientation.getW()
        << ")";

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
        << "m), Orientation: (x: "
        << pose.orientation.x
        << ", y: "
        << pose.orientation.y
        << ", z: "
        << pose.orientation.z
        << ", w: "
        << pose.orientation.w
        << ")";

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
        << quaternion.getW()
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

geometry_msgs::msg::Point Vector3ToPoint(tf2::Vector3 vector)
{
    geometry_msgs::msg::Point point;
    point.x = vector.getX();
    point.y = vector.getY();
    point.z = vector.getZ();

    return point;
}

void touchStateCallback(
    const omni_msgs::msg::OmniState::ConstSharedPtr& omniState,
    Pose& currentPose
)
{
    tf2::fromMsg(omniState->pose.position, currentPose.position);

    // Scale touch position units to metres
    currentPose.position *= TOUCH_POSITION_UNIT_SCALE_FACTOR;

    tf2::fromMsg(omniState->pose.orientation, currentPose.orientation);

    geometry_msgs::msg::Vector3 current = omniState->current;
}

void touchButtonCallback(
    const omni_msgs::msg::OmniButtonEvent::ConstSharedPtr& omniButtonStates,
    bool& isGreyButtonPressed,
    bool& isWhiteButtonPressed,
    Pose& currentUR5EPose,
    Pose& ur5eOriginPose,
    Pose& currentTouchPose,
    Pose& touchOriginPose
)
{
    // Update control origin poses if white button was just pressed
    if (omniButtonStates->white_button && !isWhiteButtonPressed)
    {
        ur5eOriginPose.position = currentUR5EPose.position;
        ur5eOriginPose.orientation = currentUR5EPose.orientation;

        touchOriginPose.position = currentTouchPose.position;
        touchOriginPose.orientation = currentTouchPose.orientation;
    }

    isGreyButtonPressed = (bool)omniButtonStates->grey_button;
    isWhiteButtonPressed = (bool)omniButtonStates->white_button;
}

bool LoadController(rclcpp::Node::SharedPtr node, const std::string controllerName)
{
    // Create a client for the load_controller service
    auto loadControllerClient = node->create_client<controller_manager_msgs::srv::LoadController>("/controller_manager/load_controller");

    // Create a service request
    auto loadController = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
    loadController->name = controllerName;

    if (!loadControllerClient->wait_for_service(5s))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call load_controller service");
        return false;
    }

    // Call the service to load the controller
    auto future = loadControllerClient->async_send_request(loadController);

    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (future.get()->ok)
        {
            RCLCPP_INFO(node->get_logger(), "Successfully loaded controller: %s", controllerName.c_str());
            return true;
        }

        RCLCPP_ERROR(node->get_logger(), "Failed to load controller: %s", controllerName.c_str());
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call load_controller service");
    }

    return false;
}

bool SwitchController(rclcpp::Node::SharedPtr node, const std::string controllerName)
{
    auto switchControllerClient = node->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller");

    auto switchController = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switchController->deactivate_controllers = std::vector<std::string>
    {
        "scaled_pos_joint_traj_controller",
        "scaled_vel_joint_traj_controller",
        "pos_joint_traj_controller",
        "vel_joint_traj_controller",
        "forward_joint_traj_controller",
        "joint_based_cartesian_traj_controller",
        "forward_cartesian_traj_controller",
        "joint_group_vel_controller",
        "twist_controller"
    };
    switchController->activate_controllers.push_back(controllerName);
    switchController->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

    if (!switchControllerClient->wait_for_service(5s))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to start controller");
        return false;
    }

    auto future = switchControllerClient->async_send_request(switchController);

    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Controller started");
        return true;
    }

    RCLCPP_ERROR(node->get_logger(), "Failed to start controller");
    return false;
}

void UpdateMovementGoal(
    FollowCartesianTrajectory::Goal& goal,
    cartesian_control_msgs::msg::CartesianTrajectoryPoint& targetPoint,
    Pose& currentUR5EPose,
    Pose& movementPoseDelta
)
{
    for (int i = 1; i <= UR5E_CONTROL_NUM_SUBDIVISIONS; i++)
    {
        targetPoint.pose.position = Vector3ToPoint(currentUR5EPose.position + movementPoseDelta.position / UR5E_CONTROL_NUM_SUBDIVISIONS);

        // Only attempt to apply the orientation delta if it is real and non-zero
        if (IsValidQuaternion(movementPoseDelta.orientation))
        {
            targetPoint.pose.orientation = tf2::toMsg(
                currentUR5EPose.orientation.slerp(
                    movementPoseDelta.orientation * currentUR5EPose.orientation,
                    UR5E_ROTATION_SCALE_FACTOR * i / UR5E_CONTROL_NUM_SUBDIVISIONS
                )
            );
        }
        // If there is no valid orientation delta, the UR5e should remain at its current orientation
        else
        {
            targetPoint.pose.orientation = tf2::toMsg(currentUR5EPose.orientation);
        }

        targetPoint.time_from_start = rclcpp::Duration::from_seconds(i * UR5E_CONTROL_TIME_INCREMENT);

        goal.trajectory.points.push_back(targetPoint);
    }
}

bool areSimilar(tf2::Quaternion a, tf2::Quaternion b)
{
    double maxDiff = 0.3;
    return (abs(a.getX() - b.getX()) < maxDiff) && (abs(a.getY() - b.getY()) < maxDiff) && (abs(a.getZ() - b.getZ()) < maxDiff);
}

int main(int argc, char **argv)
{

    Pose ur5eOriginPose;
    Pose touchOriginPose;

    Pose currentTouchPose;
    Pose currentUR5EPose;
    geometry_msgs::msg::PoseStamped currentUR5EPoseStamped;

    Pose touchPoseDelta;
    Pose ur5ePoseDelta;
    Pose movementPoseDelta;

    // Fixed transformation quaternion between touch and UR5e frames
    tf2::Quaternion touchToUR5ERotation;
    touchToUR5ERotation.setRPY(M_PI_2, 0, M_PI_2);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("ur5e_control");

    // Load and switch to the desired UR5e position controller
    LoadController(node, UR5E_CONTROLLER_NAME);

    if (!SwitchController(node, UR5E_CONTROLLER_NAME))
    {
        // Failed to switch controller, exit program
        rclcpp::shutdown();
        return -1;
    }

    FollowCartesianTrajectory::Goal goal;
    cartesian_control_msgs::msg::CartesianTrajectoryPoint targetPoint;

    auto trajectoryClient = rclcpp_action::create_client<FollowCartesianTrajectory>(node, UR5E_ACTION_CLIENT_NAME);

    bool isGreyButtonPressed = false;
    bool isWhiteButtonPressed = false;

    auto touchStateSubscriber = node->create_subscription<omni_msgs::msg::OmniState>(
        "/phantom/state",
        rclcpp::QoS(1),
        [&](const omni_msgs::msg::OmniState::ConstSharedPtr omniState)
        {
            touchStateCallback(omniState, currentTouchPose);
        }
    );

    auto touchButtonSubscriber = node->create_subscription<omni_msgs::msg::OmniButtonEvent>(
        "/phantom/button",
        rclcpp::QoS(1),
        [&](const omni_msgs::msg::OmniButtonEvent::ConstSharedPtr omniButtonStates)
        {
            touchButtonCallback(
                omniButtonStates,
                isGreyButtonPressed,
                isWhiteButtonPressed,
                currentUR5EPose,
                ur5eOriginPose,
                currentTouchPose,
                touchOriginPose
            );
        }
    );

    tf2_ros::Buffer tfBuffer(node->get_clock());
    tf2_ros::TransformListener tfListener(tfBuffer);

    while (rclcpp::ok())
    {
        try
        {
            TransformStampedToPoseStamped(tfBuffer.lookupTransform("base", "tool0_controller", tf2::TimePointZero), currentUR5EPoseStamped);

            tf2::fromMsg(currentUR5EPoseStamped.pose.position, currentUR5EPose.position);
            tf2::fromMsg(currentUR5EPoseStamped.pose.orientation, currentUR5EPose.orientation);
        }
        catch (const tf2::TransformException& exception)
        {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "%s", exception.what());
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Unable to find UR5e base to TCP transform");
            rclcpp::spin_some(node);
            continue;
        }

        // Enable UR5e teleoperation if:
        // User is pressing the white button on the Touch
        // The current pose of the UR5e is not outdated
        if (isWhiteButtonPressed &&
            (node->now() - rclcpp::Time(currentUR5EPoseStamped.header.stamp)) <= rclcpp::Duration::from_seconds(UR5E_MAX_POSE_AGE)
        )
        {
            // Update Touch position delta (in UR5E coordinate frame)
            // Touch +y-axis is aligned with UR5e -x-axis
            touchPoseDelta.position.setX(touchOriginPose.position.getY() - currentTouchPose.position.getY());
            // Touch +x-axis is aligned with UR5e -y-axis
            touchPoseDelta.position.setY(touchOriginPose.position.getX() - currentTouchPose.position.getX());
            touchPoseDelta.position.setZ(currentTouchPose.position.getZ() - touchOriginPose.position.getZ());

            // Update UR5E position delta
            ur5ePoseDelta.position = currentUR5EPose.position - ur5eOriginPose.position;

            // Final movement position delta is difference between Touch and UR5E position deltas
            movementPoseDelta.position = UR5E_TRANSLATION_SCALE_FACTOR * (touchPoseDelta.position - ur5ePoseDelta.position);

            RCLCPP_INFO_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "Current Touch Orientation: %s, Touch origin orientation: %s",
                QuaternionToString(currentTouchPose.orientation).c_str(),
                QuaternionToString(touchOriginPose.orientation).c_str()
            );

            // Update Touch orientation delta (in UR5E coordinate frame)
            touchPoseDelta.orientation = (touchToUR5ERotation * currentTouchPose.orientation) * (touchToUR5ERotation * touchOriginPose.orientation).inverse();

            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Touch orientation delta: %s", QuaternionToString(touchPoseDelta.orientation).c_str());

            RCLCPP_INFO_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "Current UR5e Orientation: %s, UR5e origin orientation: %s",
                QuaternionToString(currentUR5EPose.orientation).c_str(),
                QuaternionToString(ur5eOriginPose.orientation).c_str()
            );

            // Update UR5E orientation delta
            ur5ePoseDelta.orientation = currentUR5EPose.orientation * ur5eOriginPose.orientation.inverse();

            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "UR5e orientation delta: %s", QuaternionToString(ur5ePoseDelta.orientation).c_str());

            // Final movement orientation delta is difference between Touch and UR5E orientation deltas
            movementPoseDelta.orientation = touchPoseDelta.orientation * ur5ePoseDelta.orientation.inverse();

            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Movement pose delta: %s", PoseToString(movementPoseDelta, 1).c_str());

            for (int i = 1; i <= UR5E_CONTROL_NUM_SUBDIVISIONS; i++)
            {
                targetPoint.pose.position = Vector3ToPoint(currentUR5EPose.position + movementPoseDelta.position / UR5E_CONTROL_NUM_SUBDIVISIONS);

                // tf2::Quaternion velocityIncrement, ur5eOrientation, currentTouchOrientation, touchOriginOrientation;
                // tf2::fromMsg(currentUR5EPoseStamped.pose.orientation, ur5eOrientation);
                // tf2::fromMsg(currentTouchPoseStamped.pose.orientation, currentTouchOrientation);
                // tf2::fromMsg(TouchOriginPose.orientation, touchOriginOrientation);
                // velocityIncrement = currentTouchOrientation * touchOriginOrientation.inverse();
                // targetPoint.pose.orientation = tf2::toMsg((stylusToRobotRotation * velocityIncrement) * ur5eOrientation);
                // targetPoint.pose.orientation = StylusRotationToRobotFrame(stylusToRobotRotation, currentTouchPoseStamped.pose.orientation);

                // Only attempt to apply the orientation delta if it is real and non-zero
                if (IsValidQuaternion(movementPoseDelta.orientation))
                {
                    targetPoint.pose.orientation = tf2::toMsg(
                        currentUR5EPose.orientation.slerp(
                            movementPoseDelta.orientation * currentUR5EPose.orientation,
                            UR5E_ROTATION_SCALE_FACTOR * i / UR5E_CONTROL_NUM_SUBDIVISIONS
                        )
                    );
                }
                // If there is no valid orientation delta, the UR5e should remain at its current orientation
                else
                {
                    targetPoint.pose.orientation = currentUR5EPoseStamped.pose.orientation;
                }
                // RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Sending UR5e to Pose: %s", PoseToString(targetPoint.pose, 1).c_str());
                // RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Current UR5e Pose: %s", PoseToString(currentUR5EPose.pose, 1).c_str());

                targetPoint.time_from_start = rclcpp::Duration::from_seconds(i * UR5E_CONTROL_TIME_INCREMENT);

                goal.trajectory.points.push_back(targetPoint);
            }

            trajectoryClient->wait_for_action_server();

            bool succeeded = false;

            auto goalHandleFuture = trajectoryClient->async_send_goal(goal);

            if (rclcpp::spin_until_future_complete(node, goalHandleFuture, 5s) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto goalHandle = goalHandleFuture.get();

                if (goalHandle)
                {
                    auto resultFuture = trajectoryClient->async_get_result(goalHandle);

                    if (rclcpp::spin_until_future_complete(node, resultFuture, 5s) == rclcpp::FutureReturnCode::SUCCESS)
                    {
                        succeeded = (resultFuture.get().code == rclcpp_action::ResultCode::SUCCEEDED);
                    }
                }
            }

            goal.trajectory.points.clear();

            if (!succeeded)
            {
                RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Failed to execute trajectory or timed out");
            }
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();

    return 0;
}
