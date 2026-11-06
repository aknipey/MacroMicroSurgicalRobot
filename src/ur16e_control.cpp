#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "omni_msgs/msg/omni_state.hpp"
#include "omni_msgs/msg/omni_button_event.hpp"
#include "cartesian_control_msgs/action/follow_cartesian_trajectory.hpp"
#include "cartesian_control_msgs/msg/cartesian_trajectory_point.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "macro_micro_robot/pose.hpp"

using namespace std::chrono_literals;

// Touch position coordinates are output in millimetres, must convert to metres
const double TOUCH_POSITION_UNIT_SCALE_FACTOR = 1e-3;

// Period in seconds of each UR16e pose control operation
const double UR16E_CONTROL_PERIOD = 1e-2;

// Number of subdivisions of a single trajectory to send to the UR16E, increase number to improve motion smoothness
const double UR16E_CONTROL_NUM_SUBDIVISIONS = 10;

// The duration, in seconds, of a single pose trajectory command sent to the UR16e
const double UR16E_CONTROL_TIME_INCREMENT = UR16E_CONTROL_PERIOD / UR16E_CONTROL_NUM_SUBDIVISIONS;

// The maximum age, in seconds, of the latest pose update received from the UR16e. Past this age, control is halted until the next update.
const double UR16E_MAX_POSE_AGE = 0.1;

// Maximum tolerable uncertainty in a double
const double EPSILON = 1e-6;

// Factor by which Touch translational movements are scaled before being sent to the UR16e
const double UR16E_TRANSLATION_SCALE_FACTOR = 3;

// Factor by which Touch rotational movements are scaled before being sent to the UR16e
const double UR16E_ROTATION_SCALE_FACTOR = 0.2;

const std::string UR16E_CONTROLLER_NAME = "pose_based_cartesian_traj_controller";

const std::string UR16E_ACTION_CLIENT_NAME = "pose_based_cartesian_traj_controller/follow_cartesian_trajectory";

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
}

void touchButtonCallback(
    const omni_msgs::msg::OmniButtonEvent::ConstSharedPtr& omniButtonStates,
    bool& isGreyButtonPressed,
    bool& isWhiteButtonPressed,
    Pose& currentUR16EPose,
    Pose& ur16eOriginPose,
    Pose& currentTouchPose,
    Pose& touchOriginPose
)
{
    // Update control origin poses if white button was just pressed
    if (omniButtonStates->white_button && !isWhiteButtonPressed)
    {
        ur16eOriginPose.position = currentUR16EPose.position;
        ur16eOriginPose.orientation = currentUR16EPose.orientation;

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

int main(int argc, char **argv)
{

    Pose ur16eOriginPose;
    Pose touchOriginPose;

    Pose currentTouchPose;
    Pose currentUR16EPose;
    geometry_msgs::msg::PoseStamped currentUR16EPoseStamped;

    Pose touchPoseDelta;
    Pose ur16ePoseDelta;
    Pose movementPoseDelta;

    // Fixed transformation quaternion between touch and UR16e frames
    tf2::Quaternion touchToUR16ERotation;
    touchToUR16ERotation.setRPY(M_PI_2, 0, M_PI_2);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("ur16e_control");

    // Load and switch to the desired UR16e position controller
    LoadController(node, UR16E_CONTROLLER_NAME);

    if (!SwitchController(node, UR16E_CONTROLLER_NAME))
    {
        // Failed to switch controller, exit program
        rclcpp::shutdown();
        return -1;
    }

    FollowCartesianTrajectory::Goal goal;
    cartesian_control_msgs::msg::CartesianTrajectoryPoint targetPoint;

    auto trajectoryClient = rclcpp_action::create_client<FollowCartesianTrajectory>(node, UR16E_ACTION_CLIENT_NAME);

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
                currentUR16EPose,
                ur16eOriginPose,
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
            TransformStampedToPoseStamped(tfBuffer.lookupTransform("base", "tool0_controller", tf2::TimePointZero), currentUR16EPoseStamped);

            tf2::fromMsg(currentUR16EPoseStamped.pose.position, currentUR16EPose.position);
            tf2::fromMsg(currentUR16EPoseStamped.pose.orientation, currentUR16EPose.orientation);
        }
        catch (const tf2::TransformException& exception)
        {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "%s", exception.what());
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Unable to find UR16e base to TCP transform");
            rclcpp::spin_some(node);
            continue;
        }

        // Enable UR16e teleoperation if:
        // User is pressing the white button on the Touch
        // The current pose of the UR16e is not outdated
        if (isWhiteButtonPressed &&
            (node->now() - rclcpp::Time(currentUR16EPoseStamped.header.stamp)) <= rclcpp::Duration::from_seconds(UR16E_MAX_POSE_AGE)
        )
        {
            // Update Touch position delta (in UR16E coordinate frame)
            // Touch +y-axis is aligned with UR16e -x-axis
            touchPoseDelta.position.setX(touchOriginPose.position.getY() - currentTouchPose.position.getY());
            // Touch +x-axis is aligned with UR16e -y-axis
            touchPoseDelta.position.setY(touchOriginPose.position.getX() - currentTouchPose.position.getX());
            touchPoseDelta.position.setZ(currentTouchPose.position.getZ() - touchOriginPose.position.getZ());

            // Update UR16E position delta
            ur16ePoseDelta.position = currentUR16EPose.position - ur16eOriginPose.position;

            // Final movement position delta is difference between Touch and UR16E position deltas
            movementPoseDelta.position = UR16E_TRANSLATION_SCALE_FACTOR * (touchPoseDelta.position - ur16ePoseDelta.position);

            RCLCPP_INFO_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "Current Touch Orientation: %s, Touch origin orientation: %s",
                QuaternionToString(currentTouchPose.orientation).c_str(),
                QuaternionToString(touchOriginPose.orientation).c_str()
            );

            // Update Touch orientation delta (in UR16E coordinate frame)
            touchPoseDelta.orientation = (touchToUR16ERotation * currentTouchPose.orientation) * (touchToUR16ERotation * touchOriginPose.orientation).inverse();

            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Touch orientation delta: %s", QuaternionToString(touchPoseDelta.orientation).c_str());

            RCLCPP_INFO_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "Current UR16e Orientation: %s, UR16e origin orientation: %s",
                QuaternionToString(currentUR16EPose.orientation).c_str(),
                QuaternionToString(ur16eOriginPose.orientation).c_str()
            );

            // Update UR16E orientation delta
            ur16ePoseDelta.orientation = currentUR16EPose.orientation * ur16eOriginPose.orientation.inverse();

            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "UR16e orientation delta: %s", QuaternionToString(ur16ePoseDelta.orientation).c_str());

            // Final movement orientation delta is difference between Touch and UR16E orientation deltas
            movementPoseDelta.orientation = touchPoseDelta.orientation * ur16ePoseDelta.orientation.inverse();

            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Movement pose delta: %s", PoseToString(movementPoseDelta, 1).c_str());

            for (int i = 1; i <= UR16E_CONTROL_NUM_SUBDIVISIONS; i++)
            {
                targetPoint.pose.position = Vector3ToPoint(currentUR16EPose.position + movementPoseDelta.position / UR16E_CONTROL_NUM_SUBDIVISIONS);

                // Only attempt to apply the orientation delta if it is real and non-zero
                if (IsValidQuaternion(movementPoseDelta.orientation))
                {
                    targetPoint.pose.orientation = tf2::toMsg(
                        currentUR16EPose.orientation.slerp(
                            movementPoseDelta.orientation * currentUR16EPose.orientation,
                            UR16E_ROTATION_SCALE_FACTOR * i / UR16E_CONTROL_NUM_SUBDIVISIONS
                        )
                    );
                }
                // If there is no valid orientation delta, the UR16e should remain at its current orientation
                else
                {
                    targetPoint.pose.orientation = currentUR16EPoseStamped.pose.orientation;
                }

                targetPoint.time_from_start = rclcpp::Duration::from_seconds(i * UR16E_CONTROL_TIME_INCREMENT);

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
