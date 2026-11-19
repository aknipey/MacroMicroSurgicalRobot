#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
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
#include "tf2_ros/transform_listener.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "macro_micro_robot/ManipulatorModule.hpp"
#include "macro_micro_robot/Pose.hpp"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include "motor_angles_msg/msg/motor_angles.hpp"

// Touch position coordinates are output in millimetres, must convert to metres
const double TOUCH_POSITION_UNIT_SCALE_FACTOR = 1e-3;

// Share of the way from the tip's orientation to the pen's (1:1 tilt) taken each 60 Hz
// cycle: a first-order lag, not a scale. 0.02 was a ~0.8 s time constant; 0.2 is ~80 ms.
const double MANIPULATOR_ROTATION_SCALE_FACTOR = 0.2;

// Tip motion per unit of stylus motion. The module is a few millimetres long, so the
// stylus is scaled down: 40 mm of hand motion moves the tip 2 mm. (It was 10x up: a
// 40 mm stroke asked for a tip target 0.4 m away.)
const double MICRO_POSITION_SCALE = 0.05;

// Position errors (m) are divided by this length before the inverse kinematics, so they
// weigh about as much as orientation errors (rad): the tip's lever arms are millimetres,
// so unweighted the position rows of the Jacobian are ~1/200 of the angle rows and the
// damping swamped them. ~ joint-to-tip lever.
const double POSITION_WEIGHT_LENGTH = 0.005;

// Maximum (weighted) pose step for a single update: rad, or POSITION_WEIGHT_LENGTH x this in metres
const double JOINT_UPDATE_MAX_VELOCITY = 0.1;

// Maximum error in end-effector pose, in metres
const double MAX_POSE_ERROR = 0.00001;

// Diameter of the manipulator, in metres
const double MANIPULATOR_DIAMETER = 0.004;

// Offset of end-effector tip from the centre of the final joint
const double END_EFFECTOR_TIP_OFFSET = 0.00244;

// Radius of the actuator pulley wheels, in metres
const double PULLEY_WHEEL_RADIUS = 0.01125;

// Conversion factor between a tendon length delta in metres and a pulley wheel rotation in degrees
const double LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES = 180 / M_PI / PULLEY_WHEEL_RADIUS;

// Centre-to-centre distance between tendon pairs in the cross-section of a sub-module, in metres
const double TENDON_PAIR_SEPARATION = 0.003;

// Half angle of curvature of a proximal sub-module, in radians
const double PROXIMAL_HALF_CURVATURE_ANGLE = 0.2;

// Number of joints in the proximal module
const int PROXIMAL_NUM_JOINTS = 3;

// Separation between rolling surfaces in a proximal sub-module, in radians
const double PROXIMAL_JOINT_SEPARATION = 0.001;

// Length of a proximal sub-module, in metres
const double PROXIMAL_LENGTH = 0.0014;

// Half angle of curvature of a proximal sub-module, in radians
const double DISTAL_HALF_CURVATURE_ANGLE = 0.88;

// Number of joints in the distal module
const int DISTAL_NUM_JOINTS = 3;

// Separation between rolling surfaces in a distal sub-module, in radians
const double DISTAL_JOINT_SEPARATION = 0.001;

// Length of a distal sub-module, in metres
const double DISTAL_LENGTH = 0.00288;

// lambda in the damped least squares inverse, against the weighted Jacobian (O(1)). D
// grows from 1 mid-range to 2 at a joint limit, so the damping still rises near limits.
// Lower = more responsive; was 1 (with a stray extra D^2: effective 2).
const double LEAST_SQUARES_DAMPING_FACTOR = 0.3;

typedef std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > TransformVector;

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

void PrintMatrix(const Eigen::MatrixXd& matrix, const char* title)
{
    std::stringstream output;

    output << matrix;

    RCLCPP_INFO(rclcpp::get_logger("micro_module_control"), "%s:\n%s", title, output.str().c_str());
}

Eigen::Matrix3d xRotationMatrix(double theta)
{
    Eigen::Matrix3d rotation;

    rotation << 1, 0, 0,
                0, cos(theta), -sin(theta),
                0, sin(theta), cos(theta);

    return rotation;
}

Eigen::Matrix3d yRotationMatrix(double theta)
{
    Eigen::Matrix3d rotation;

    rotation << cos(theta), 0, sin(theta),
                0, 1, 0,
                -sin(theta), 0, cos(theta);

    return rotation;
}

Eigen::Matrix4d zRotationTransform(double theta)
{
    Eigen::Matrix4d transform;

    transform << cos(theta), -sin(theta), 0.0, 0.0,
                 sin(theta), cos(theta), 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0;

    return transform;
}

Eigen::Matrix4d TransformFromPosition(const Eigen::Vector3d& position)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    transform.block(0, 3, 3, 1) = position;

    return transform;
}

Eigen::Matrix4d TransformFromRotation(const Eigen::Matrix3d& rotation)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    transform.block(0, 0, 3, 3) = rotation;

    return transform;
}

Eigen::Matrix4d TransformFromPose(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    transform.block(0, 0, 3, 3) = rotation;
    transform.block(0, 3, 3, 1) = position;

    return transform;
}

// Extract the position vector from a homogeneous transform
Eigen::Vector3d GetTransformPosition(const Eigen::Matrix4d& transform)
{
    Eigen::Vector3d position;
    position << transform.block(0, 3, 3, 1);

    return position;
}

Eigen::Matrix3d GetTransformRotation(const Eigen::Matrix4d& transform)
{
    Eigen::Matrix3d rotation;
    rotation = transform.block(0, 0, 3, 3);

    return rotation;
}

TransformVector GetJointTransforms(const Eigen::Matrix4d& baseTransform, const Eigen::Matrix4d toolTransform, const ManipulatorModule& proximal, const ManipulatorModule& distal)
{
    std::vector<ManipulatorModule> modules {proximal, distal};

    Eigen::Matrix4d moduleInterfaceAngularOffset = zRotationTransform(-M_PI_4);

    Eigen::Matrix4d endTransform = baseTransform;

    Eigen::Matrix4d panJointTransform = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d tiltJointTransform = Eigen::Matrix4d::Zero();

    TransformVector jointTransforms;

    for (std::size_t i = 0; i < modules.size(); ++i)
    {
        panJointTransform = modules[i].GetPanJointTransform();
        tiltJointTransform = modules[i].GetTiltJointTransform();

        // PrintMatrix(panJointTransform, "Pan Joint Transform");
        // PrintMatrix(tiltJointTransform, "Tilt Joint Transform");

        for (int j = 0; j < modules[i].GetNumJoints(); j++)
        {
            // Current joint is a pan-type if the first joint in the module is pan and the joint index is even, or if the inverse is true
            if (modules[i].IsFirstJointPan() ^ (j % 2 != 0))
            {
                endTransform *= panJointTransform;
            }
            else
            {
                endTransform *= tiltJointTransform;
            }

            // PrintMatrix(endTransform, "Pushing back matrix: ");

            Eigen::Matrix4d jointTransform = endTransform;

            jointTransforms.push_back(jointTransform);
        }

        // Modules are offset by a 45-degree rotation at their interface
        if (i != modules.size() - 1)
        {
            endTransform *= moduleInterfaceAngularOffset;
            // PrintMatrix(endTransform, "Rotated end transform");
        }
    }

    endTransform *= toolTransform;

    jointTransforms.push_back(endTransform);

    // PrintMatrix(endTransform, "End Transform");

    return jointTransforms;
}

Eigen::MatrixXd GetJacobian(const TransformVector& jointTransforms, const ManipulatorModule& proximal, const ManipulatorModule& distal)
{
    std::vector<ManipulatorModule> modules {proximal, distal};

    Eigen::Vector3d xAxis;
    xAxis << 1, 0, 0;

    Eigen::Vector3d yAxis;
    yAxis << 0, 1, 0;

    // The final Jacobian will have 6 rows and 2 * {num. modules} columns
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 2 * modules.size());

    Eigen::VectorXd jointJacobianColumn(6);
    Eigen::Vector3d jointCrossProduct;

    Eigen::VectorXd modulePanJacobianColumn;
    Eigen::VectorXd moduleTiltJacobianColumn;

    Eigen::Vector3d endPosition = GetTransformPosition(jointTransforms.back());

    // The number of joint transforms applied to the Jacobian
    int numJointTransformsProcessed = 0;

    for (std::size_t i = 0; i < modules.size(); ++i)
    {
        modulePanJacobianColumn = Eigen::VectorXd::Zero(6);
        moduleTiltJacobianColumn = Eigen::VectorXd::Zero(6);

        for (int j = 0; j < modules[i].GetNumJoints(); j++)
        {
            Eigen::Matrix4d jointTransform = jointTransforms[numJointTransformsProcessed + j];
            // PrintMatrix(jointTransform, "Joint Transform");
            // Current joint is a pan-type if the first joint in the module is pan and the joint index is even, or if the inverse is true
            if (modules[i].IsFirstJointPan() ^ (j % 2 != 0))
            {
                Eigen::Vector3d actuationAxis = GetTransformRotation(jointTransform) * xAxis;
                jointCrossProduct = actuationAxis.cross(endPosition - GetTransformPosition(jointTransform));

                // PrintMatrix(jointCrossProduct, "Joint cross product");

                jointJacobianColumn << jointCrossProduct, actuationAxis;

                modulePanJacobianColumn += jointJacobianColumn;
            }
            else
            {
                Eigen::Vector3d actuationAxis = GetTransformRotation(jointTransform) * yAxis;
                jointCrossProduct = actuationAxis.cross(endPosition - GetTransformPosition(jointTransform));

                // PrintMatrix(jointCrossProduct, "Joint cross product");

                jointJacobianColumn << jointCrossProduct, actuationAxis;

                moduleTiltJacobianColumn += jointJacobianColumn;
            }
        }

        modulePanJacobianColumn /= modules[i].GetNumPanJoints();
        moduleTiltJacobianColumn /= modules[i].GetNumTiltJoints();

        // PrintMatrix(modulePanJacobianColumn, "Pan Jacobian column post-division");
        // PrintMatrix(moduleTiltJacobianColumn, "Tilt Jacobian column post-division");

        if (modules[i].IsFirstJointPan())
        {
            jacobian.col(2 * i) = modulePanJacobianColumn;
            jacobian.col(2 * i + 1) = moduleTiltJacobianColumn;
        }
        else
        {
            jacobian.col(2 * i) = moduleTiltJacobianColumn;
            jacobian.col(2 * i + 1) = modulePanJacobianColumn;
        }

        numJointTransformsProcessed += modules[i].GetNumJoints();
    }

    return jacobian;
}

// Obtain an approximation of the inverse Jacobian using the damped least squares method
Eigen::MatrixXd DampedLeastSquares(Eigen::MatrixXd jacobian, double leastSquaresDampingFactor)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    Eigen::VectorXd singularValues = svd.singularValues();

    Eigen::MatrixXd inverseJacobian = Eigen::MatrixXd::Zero(jacobian.cols(), jacobian.rows());

    for (int i = 0; i < svd.nonzeroSingularValues(); i++)
    {
        inverseJacobian += singularValues(i) / (pow(singularValues(i), 2) + pow(leastSquaresDampingFactor, 2)) * V.col(i) * U.col(i).transpose();
    }

    return inverseJacobian;
}

// Obtain an approximation of the inverse Jacobian using the damped least squares method
Eigen::MatrixXd GetInverseJacobian(Eigen::MatrixXd& jacobian, double leastSquaresDampingFactor)
{
    Eigen::MatrixXd inverseJacobian(jacobian.cols(), jacobian.rows());

    inverseJacobian = (jacobian.transpose() * jacobian + Eigen::Matrix4d::Identity()).colPivHouseholderQr().solve(jacobian.transpose());

    return inverseJacobian;
}

// Obtain an approximation of the inverse Jacobian using the damped least squares method
Eigen::MatrixXd GetInverseJacobianDamped(
    Eigen::MatrixXd& jacobian,
    const Eigen::Vector4d& jointPositions,
    const Eigen::Vector4d& minJointAngles,
    const Eigen::Vector4d& maxJointAngles,
    double leastSquaresDampingFactor
)
{
    Eigen::Matrix4d D = Eigen::Matrix4d::Identity();

    double minAngle, maxAngle;

    for (int i = 0; i < jointPositions.size(); i++)
    {
        minAngle = minJointAngles(i);
        maxAngle = maxJointAngles(i);

        D(i, i) = pow((2 * jointPositions(i) - maxAngle - minAngle) / (maxAngle - minAngle), 2) + 1;
    }

    // (Joint positions, limits, lambda and D were printed here every cycle at 60 Hz,
    // flooding the launch terminal; the limits are logged once at startup instead)

    Eigen::MatrixXd inverseJacobian(jacobian.cols(), jacobian.rows());

    // Damped least squares (J^T J + lambda^2 D^2)^-1 J^T. There was an extra unscaled D^2
    // term, so with lambda = 1 the damping was 2 D^2 and each cycle covered only a small
    // part of the error (a 1.5 mm tip step took 1.1 s to settle)
    inverseJacobian = (jacobian.transpose() * jacobian +
    leastSquaresDampingFactor * leastSquaresDampingFactor * D * D).colPivHouseholderQr().solve(jacobian.transpose());

    return inverseJacobian;
}

// Apply a total-angle delta to joint i (0 proximal pan, 1 proximal tilt, 2 distal pan, 3 distal tilt)
void ApplyJointDelta(ManipulatorModule& proximal, ManipulatorModule& distal, int i, double delta)
{
    switch (i)
    {
        case 0: proximal.ApplyPanAngleDelta(delta); break;
        case 1: proximal.ApplyTiltAngleDelta(delta); break;
        case 2: distal.ApplyPanAngleDelta(delta); break;
        default: distal.ApplyTiltAngleDelta(delta); break;
    }
}

bool MotorsWithinLimits(const Eigen::Vector4d& motors, const Eigen::VectorXd& motorMin, const Eigen::VectorXd& motorMax)
{
    for (int i = 0; i < 4; i++)
    {
        if (motors(i) < motorMin(i) || motors(i) > motorMax(i))
        {
            return false;
        }
    }
    return true;
}

Eigen::Vector4d GetMotorPositionsFromJointPositions(const ManipulatorModule& proximal, const ManipulatorModule& distal);

// Per-joint angle range (rad) over which every motor stays inside its limits, moving one
// joint at a time from the current (zero) joint positions, for the damped inverse's joint
// limits. Steps through the tendon model, so it follows its nonlinearity and the distal
// tendons' coupling to the proximal joints.
void GetJointRangeFromMotorLimits(
    const ManipulatorModule& proximal,
    const ManipulatorModule& distal,
    const Eigen::VectorXd& motorReference,
    const Eigen::VectorXd& motorMin,
    const Eigen::VectorXd& motorMax,
    Eigen::Vector4d& minJointAngles,
    Eigen::Vector4d& maxJointAngles
)
{
    const double step = 0.001;
    const double maxTotalAngle = 2.0;
    const int jointsOfType[4] = {
        proximal.GetNumPanJoints(), proximal.GetNumTiltJoints(), distal.GetNumPanJoints(), distal.GetNumTiltJoints()
    };

    for (int i = 0; i < 4; i++)
    {
        for (int direction = -1; direction <= 1; direction += 2)
        {
            ManipulatorModule p = proximal, d = distal;
            double total = 0.0;
            while (std::abs(total) < maxTotalAngle)
            {
                ApplyJointDelta(p, d, i, direction * step);
                if (!MotorsWithinLimits(motorReference + GetMotorPositionsFromJointPositions(p, d), motorMin, motorMax))
                {
                    break;
                }
                total += direction * step;
            }
            (direction < 0 ? minJointAngles : maxJointAngles)(i) = total / jointsOfType[i];
        }
    }
}

Eigen::Vector4d GetMotorPositionsFromJointPositions(const ManipulatorModule& proximal, const ManipulatorModule& distal)
{
    Eigen::Vector4d motorPositions = Eigen::Vector4d::Zero();

    // RCLCPP_INFO(rclcpp::get_logger("micro_module_control"), "test: %.6f", test);

    double proximalPanLengthDelta = proximal.GetIsolatedPanLengthDelta();
    double proximalTiltLengthDelta = proximal.GetIsolatedTiltLengthDelta();

    // RCLCPP_INFO(rclcpp::get_logger("micro_module_control"), "prox. pan: %.6f, prox. tilt: %.6f", proximalPanLengthDelta, proximalTiltLengthDelta);

    motorPositions(0) = proximalPanLengthDelta * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;
    motorPositions(1) = proximalTiltLengthDelta * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;

    // The difference in tendon length change resulting from the angular offset of the distal tendons on the proximal joints
    double distalTendonProximalJointOffset = (2 - sqrt(2)) * proximal.GetCurvatureRadius() * sin(proximal.GetHalfCurvatureAngle()) * (
        proximal.GetNumPanJoints() * sin(proximal.GetPanJointAngle() / 2) +
        proximal.GetNumTiltJoints() * sin(proximal.GetTiltJointAngle() / 2)
    );

    // The contribution of the proximal joint positions to the length delta of the distal tendons
    double distalLengthDeltaDueToProximalJoints = proximalPanLengthDelta + proximalTiltLengthDelta + distalTendonProximalJointOffset;

    // RCLCPP_INFO(rclcpp::get_logger("micro_module_control"), "dLengthDelta: %.6f", distalLengthDeltaDueToProximalJoints);

    motorPositions(2) = (distal.GetIsolatedPanLengthDelta() + distalLengthDeltaDueToProximalJoints) * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;
    motorPositions(3) = (distal.GetIsolatedTiltLengthDelta() + distalLengthDeltaDueToProximalJoints) * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;

    // RCLCPP_INFO(rclcpp::get_logger("micro_module_control"), "dist. pan: %.6f, dist. tilt: %.6f", motorPositions(0), motorPositions(1));

    return motorPositions;
}

Eigen::Vector4d GetJointPositionsFromMotorPositions(const Eigen::Vector4d& motorStates, const ManipulatorModule& proximal, const ManipulatorModule& distal)
{
    Eigen::Vector4d jointPositions = Eigen::Vector4d::Zero();

    // Ratio of proximal curvature radius to pulley radius
    double proximalRadiusRatio = proximal.GetCurvatureRadius() / PULLEY_WHEEL_RADIUS;
    double proximalHalfPanJointAngle = proximal.GetPanJointAngle() / 2;
    double proximalHalfTiltJointAngle = proximal.GetTiltJointAngle() / 2;

    // Construct proximal actuator Jacobian
    Eigen::Matrix2d proximalJacobian;
    proximalJacobian << -proximalRadiusRatio * sin(proximal.GetHalfCurvatureAngle() - proximalHalfPanJointAngle), proximalRadiusRatio * sin(proximalHalfTiltJointAngle),
                         proximalRadiusRatio * sin(proximalHalfPanJointAngle), -proximalRadiusRatio * sin(proximal.GetHalfCurvatureAngle() - proximalHalfTiltJointAngle);

    // PrintMatrix(proximalJacobian, "Proximal Actuator Jacobian");

    double distalRadiusRatio = distal.GetCurvatureRadius() / PULLEY_WHEEL_RADIUS;
    double distalHalfPanJointAngle = distal.GetPanJointAngle() / 2;
    double distalHalfTiltJointAngle = distal.GetTiltJointAngle() / 2;

    // Constant coefficient of partial derivatives of distal tendon lengths w.r.t. proximal joint angles
    double derivativeCoefficient = 1 - 1 / sqrt(2);

    // Construct distal actuator Jacobian
    Eigen::MatrixXd distalJacobian(2, 4);
    distalJacobian <<  proximalRadiusRatio * derivativeCoefficient * sin(proximal.GetHalfCurvatureAngle()) * cos(proximalHalfPanJointAngle),
                       proximalRadiusRatio * derivativeCoefficient * sin(proximal.GetHalfCurvatureAngle()) * cos(proximalHalfTiltJointAngle),
                       -distalRadiusRatio * sin(distal.GetHalfCurvatureAngle() - distalHalfPanJointAngle),
                       distalRadiusRatio * sin(distalHalfTiltJointAngle),
                       proximalRadiusRatio * derivativeCoefficient * sin(proximal.GetHalfCurvatureAngle()) * cos(proximalHalfPanJointAngle),
                       proximalRadiusRatio * derivativeCoefficient * sin(proximal.GetHalfCurvatureAngle()) * cos(proximalHalfTiltJointAngle),
                       distalRadiusRatio * sin(distalHalfPanJointAngle),
                       -distalRadiusRatio * sin(distal.GetHalfCurvatureAngle() - distalHalfTiltJointAngle);

    // PrintMatrix(distalJacobian, "Distal Actuator Jacobian");

    // Combine proximal and distal Jacobians. Note that the distal motor angles have no effect on the proximal joint positions
    Eigen::Matrix4d combinedJacobian = Eigen::Matrix4d::Zero();
    combinedJacobian.block(0, 0, 2, 2) = proximalJacobian;
    combinedJacobian.block(2, 0, 2, 4) = distalJacobian;

    // PrintMatrix(combinedJacobian, "Combined Jacobian");

    jointPositions = combinedJacobian * motorStates;

    return jointPositions;
}

Eigen::Vector3d SkewSymmetricMatrixToVector(const Eigen::Matrix3d& matrix)
{
    // Creates a vector v from a skew-symmetric matrix, assuming the following matrix layout:
    // |   0 -vz  vy |
    // |  vz   0 -vx |
    // | -vy  vx   0 |
    Eigen::Vector3d vector;
    vector << -matrix(1, 2), matrix(0, 2), -matrix(0, 1);
    return vector;
}

Eigen::VectorXd GetPoseDelta(const Eigen::Vector3d& currentPosition, const Eigen::Vector3d& desiredPosition, const Eigen::Matrix3d& currentRotation, const Eigen::Matrix3d& desiredRotation)
{
    Eigen::VectorXd poseDelta(6);
    poseDelta << (desiredPosition - currentPosition), SkewSymmetricMatrixToVector(desiredRotation * currentRotation.inverse() - Eigen::Matrix3d::Identity());

    return poseDelta;
}

Eigen::VectorXd CapVectorMagnitude(const Eigen::VectorXd& vector, const double maxMagnitude)
{
    Eigen::VectorXd cappedVector = maxMagnitude * vector.normalized();

    return cappedVector;
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
    bool& isWhiteButtonPressed
)
{
    isGreyButtonPressed = (bool)omniButtonStates->grey_button;
    isWhiteButtonPressed = (bool)omniButtonStates->white_button;
}

tf2::Quaternion EigenRotationMatrixToTF2Quaternion(const Eigen::Matrix3d& rotation)
{
    Eigen::Quaterniond eigenQuat(rotation);
    tf2::Quaternion tf2Quat = tf2::Quaternion(eigenQuat.x(), eigenQuat.y(), eigenQuat.z(), eigenQuat.w());

    return tf2Quat;
}

Eigen::Matrix3d tf2QuaternionToEigenRotationMatrix(const tf2::Quaternion& quaternion)
{
    Eigen::Quaterniond eigenQuat(quaternion.getW(), quaternion.getX(), quaternion.getY(), quaternion.getZ());

    Eigen::Matrix3d rotationMatrix = eigenQuat.toRotationMatrix();

    return rotationMatrix;
}

void motorStatesCallback(
    const motor_angles_msg::msg::MotorAngles::ConstSharedPtr& motorStates,
    Eigen::VectorXd& motorStatesCache,
    Eigen::VectorXd& motorCommands,
    bool& hasReceivedMotorStates
)
{
    motorStatesCache << motorStates->proximal_pan_angle, motorStates->proximal_tilt_angle, motorStates->distal_pan_angle, motorStates->distal_tilt_angle;

    if (!hasReceivedMotorStates)
    {
        motorCommands = motorStatesCache;
        PrintMatrix(motorCommands, "Setting motor commands to");
    }

    hasReceivedMotorStates = true;
}

void motorMinAnglesCallback(const motor_angles_msg::msg::MotorAngles::ConstSharedPtr& motorMinAngles, Eigen::VectorXd& motorMinAnglesCache, bool& hasReceivedMotorMinAngles)
{
    hasReceivedMotorMinAngles = true;
    motorMinAnglesCache << motorMinAngles->proximal_pan_angle, motorMinAngles->proximal_tilt_angle, motorMinAngles->distal_pan_angle, motorMinAngles->distal_tilt_angle;
}

void motorMaxAnglesCallback(const motor_angles_msg::msg::MotorAngles::ConstSharedPtr& motorMaxAngles, Eigen::VectorXd& motorMaxAnglesCache, bool& hasReceivedMotorMaxAngles)
{
    hasReceivedMotorMaxAngles = true;
    motorMaxAnglesCache << motorMaxAngles->proximal_pan_angle, motorMaxAngles->proximal_tilt_angle, motorMaxAngles->distal_pan_angle, motorMaxAngles->distal_tilt_angle;
}

double clamp(double value, double min, double max)
{
    return std::min(std::max(value, min), max);
}

int main(int argc, char **argv)
{
    Pose touchOriginPose;
    Eigen::Matrix3d endOriginRotation;

    Eigen::Matrix3d endRotation;
    Pose currentTouchPose;
    geometry_msgs::msg::PoseStamped currentUR16EPoseStamped;

    Pose touchPoseDelta;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("micro_module_control");

    auto touchStateSubscriber = node->create_subscription<omni_msgs::msg::OmniState>(
        "/phantom/state",
        rclcpp::QoS(1),
        [&](const omni_msgs::msg::OmniState::ConstSharedPtr omniState)
        {
            touchStateCallback(omniState, currentTouchPose);
        }
    );

    bool isGreyButtonPressed = false;
    bool isWhiteButtonPressed = false;
    bool wasEngaged = false;  // Track previous engaged state for edge detection

    // Which module the stylus drives, published latched by ur16e_touch_teleop (hold GREY
    // 1 s to switch). Without that node running nothing is published and the micro
    // module is always the active one.
    bool isMicroModeActive = true;

    bool hasReceivedMotorStates = false;
    bool hasReceivedMotorMinAngles = false;
    bool hasReceivedMotorMaxAngles = false;

    Eigen::VectorXd motorStates(4);
    Eigen::VectorXd motorCommands(4);
    Eigen::VectorXd motorMinAngles(4);
    Eigen::VectorXd motorMaxAngles(4);

    auto touchButtonSubscriber = node->create_subscription<omni_msgs::msg::OmniButtonEvent>(
        "/phantom/button",
        rclcpp::QoS(1),
        [&](const omni_msgs::msg::OmniButtonEvent::ConstSharedPtr omniButtonStates)
        {
            touchButtonCallback(
                omniButtonStates,
                isGreyButtonPressed,
                isWhiteButtonPressed
            );
        }
    );

    auto teleopModeSubscriber = node->create_subscription<std_msgs::msg::String>(
        "/teleop_mode",
        rclcpp::QoS(1).transient_local(),
        [&](const std_msgs::msg::String::ConstSharedPtr modeMsg)
        {
            bool isMicro = modeMsg->data == "micro";
            if (isMicro != isMicroModeActive)
            {
                RCLCPP_INFO(node->get_logger(), isMicro
                    ? "Teleop mode MICRO - hold WHITE to drive the micro-manipulator"
                    : "Teleop mode MACRO - micro-manipulator holding still");
            }
            isMicroModeActive = isMicro;
        }
    );

    // Transient local, as serial_bridge publishes it: the Arduino only reports in reply to
    // a command, so a state published before this node started would otherwise be lost
    // and neither side would ever send first
    auto motorStatesSubscriber = node->create_subscription<motor_angles_msg::msg::MotorAngles>(
        "micro_module_motor_states",
        rclcpp::QoS(1).transient_local(),
        [&](const motor_angles_msg::msg::MotorAngles::ConstSharedPtr motorStatesMsg)
        {
            motorStatesCallback(
                motorStatesMsg,
                motorStates,
                motorCommands,
                hasReceivedMotorStates
            );
        }
    );

    auto motorMinAnglesSubscriber = node->create_subscription<motor_angles_msg::msg::MotorAngles>(
        "micro_module_motor_min_angles",
        rclcpp::QoS(1),
        [&](const motor_angles_msg::msg::MotorAngles::ConstSharedPtr motorMinAnglesMsg)
        {
            motorMinAnglesCallback(
                motorMinAnglesMsg,
                motorMinAngles,
                hasReceivedMotorMinAngles
            );
        }
    );

    auto motorMaxAnglesSubscriber = node->create_subscription<motor_angles_msg::msg::MotorAngles>(
        "micro_module_motor_max_angles",
        rclcpp::QoS(1),
        [&](const motor_angles_msg::msg::MotorAngles::ConstSharedPtr motorMaxAnglesMsg)
        {
            motorMaxAnglesCallback(
                motorMaxAnglesMsg,
                motorMaxAngles,
                hasReceivedMotorMaxAngles
            );
        }
    );

    auto motorCommandsPublisher = node->create_publisher<motor_angles_msg::msg::MotorAngles>("micro_module_motor_command", rclcpp::QoS(1));

    ManipulatorModule proximal(
        true,
        MANIPULATOR_DIAMETER,
        PROXIMAL_HALF_CURVATURE_ANGLE,
        PROXIMAL_NUM_JOINTS,
        PROXIMAL_JOINT_SEPARATION
    );

    ManipulatorModule distal(
        true,
        MANIPULATOR_DIAMETER,
        DISTAL_HALF_CURVATURE_ANGLE,
        DISTAL_NUM_JOINTS,
        DISTAL_JOINT_SEPARATION
    );

    // Offset of end effector tip from centre of final manipulator joint
    Eigen::Vector3d endEffectorTipPositionOffset;
    endEffectorTipPositionOffset << 0, 0, END_EFFECTOR_TIP_OFFSET;

    Eigen::Matrix4d endEffectorTransform = TransformFromPosition(endEffectorTipPositionOffset);

    PrintMatrix(endEffectorTransform, "End Effector");

    // Wait for all required data to be received
    while (!hasReceivedMotorStates || !hasReceivedMotorMinAngles || !hasReceivedMotorMaxAngles)
    {
        rclcpp::spin_some(node);
    }
    
    RCLCPP_INFO(node->get_logger(), "Received motor states: %.6f, %.6f, %.6f, %.6f", 
        motorStates(0), motorStates(1), motorStates(2), motorStates(3));
    RCLCPP_INFO(node->get_logger(), "Received motor min angles: %.6f, %.6f, %.6f, %.6f", 
        motorMinAngles(0), motorMinAngles(1), motorMinAngles(2), motorMinAngles(3));
    RCLCPP_INFO(node->get_logger(), "Received motor max angles: %.6f, %.6f, %.6f, %.6f", 
        motorMaxAngles(0), motorMaxAngles(1), motorMaxAngles(2), motorMaxAngles(3));

    // Joint angles 0 = the motor angles the Arduino reported at startup (its equilibrium
    // after a reset). From then on every motor command is that reference plus the
    // tendon model's motor angle for the current joint angles.
    proximal.SetTotalPanAngle(0.0);
    proximal.SetTotalTiltAngle(0.0);
    distal.SetTotalPanAngle(0.0);
    distal.SetTotalTiltAngle(0.0);
    const Eigen::VectorXd motorReference = motorStates;

    Eigen::Matrix4d baseTransform = Eigen::Matrix4d::Identity();

    // Initialize end effector origin rotation to current rotation
    TransformVector initialJointTransforms = GetJointTransforms(baseTransform, endEffectorTransform, proximal, distal);
    endOriginRotation = GetTransformRotation(initialJointTransforms.back());

    RCLCPP_INFO(node->get_logger(), "Proximal parameters:\n\tcurvature radius: %.6f\n\tnum pan joints: %d\n\thalf curvature angle: %.6f\n\tpan joint angle: %.6f\n\tnum tilt joints: %d\n\ttilt joint angle: %.6f\n\tpan central separation: %.6f\n\ttilt central separation: %.6f\n\tjoint separation distance: %.6f\n\tisolated pan length delta: %.6f\n\tisolated tilt length delta: %.6f",
        proximal.GetCurvatureRadius(),
        proximal.GetNumPanJoints(),
        proximal.GetHalfCurvatureAngle(),
        proximal.GetPanJointAngle(),
        proximal.GetNumTiltJoints(),
        proximal.GetTiltJointAngle(),
        proximal.GetPanCentralSeparation(),
        proximal.GetTiltCentralSeparation(),
        proximal.GetJointSeparationDistance(),
        proximal.GetIsolatedPanLengthDelta(),
        proximal.GetIsolatedTiltLengthDelta()
    );

    RCLCPP_INFO(node->get_logger(), "Distal parameters:\n\tcurvature radius: %.6f\n\tnum pan joints: %d\n\thalf curvature angle: %.6f\n\tpan joint angle: %.6f\n\tnum tilt joints: %d\n\ttilt joint angle: %.6f\n\tpan central separation: %.6f\n\ttilt central separation: %.6f\n\tjoint separation distance: %.6f\n\tisolated pan length delta: %.6f\n\tisolated tilt length delta: %.6f",
        distal.GetCurvatureRadius(),
        distal.GetNumPanJoints(),
        distal.GetHalfCurvatureAngle(),
        distal.GetPanJointAngle(),
        distal.GetNumTiltJoints(),
        distal.GetTiltJointAngle(),
        distal.GetPanCentralSeparation(),
        distal.GetTiltCentralSeparation(),
        distal.GetJointSeparationDistance(),
        proximal.GetIsolatedPanLengthDelta(),
        proximal.GetIsolatedTiltLengthDelta()
    );

    // Fixed transformation quaternion between touch and manipulator frames
    tf2::Quaternion touchToManipulatorRotation;
    // touchToManipulatorRotation.setRPY(0, M_PI, M_PI_4);
    touchToManipulatorRotation.setRPY(0, 0, M_PI_4);

    Eigen::Vector3d endPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d endOriginPosition = Eigen::Vector3d::Zero();

    tf2::Quaternion manipulatorOrientation;
    tf2::Quaternion manipulatorOriginOrientation;
    tf2::Quaternion manipulatorOrientationDelta;

    tf2::Quaternion movementOrientationDelta;

    Eigen::Matrix3d movementRotation;

    // Joint limits for the damped inverse: where a motor would reach its limit. These were
    // fixed at +-0.05 / +-0.025 rad, which the tendon model puts at only ~1 deg of motor
    // travel (of 11-22 deg), so the damping held the motors within about a degree of rest.
    Eigen::Vector4d minJointAngles;
    Eigen::Vector4d maxJointAngles;
    GetJointRangeFromMotorLimits(proximal, distal, motorReference, motorMinAngles, motorMaxAngles,
                                 minJointAngles, maxJointAngles);
    RCLCPP_INFO(node->get_logger(), "Joint limits from the motor limits (rad per joint): "
        "prox. pan %.3f..%.3f, prox. tilt %.3f..%.3f, dist. pan %.3f..%.3f, dist. tilt %.3f..%.3f",
        minJointAngles(0), maxJointAngles(0), minJointAngles(1), maxJointAngles(1),
        minJointAngles(2), maxJointAngles(2), minJointAngles(3), maxJointAngles(3));

    Eigen::Vector4d jointPositions;
    jointPositions << proximal.GetPanJointAngle(), proximal.GetTiltJointAngle(), distal.GetPanJointAngle(), distal.GetTiltJointAngle();

    rclcpp::Rate rate(60);

    while (rclcpp::ok())
    {
        // Process callbacks first to get latest sensor data
        rclcpp::spin_some(node);

        TransformVector jointTransforms = GetJointTransforms(baseTransform, endEffectorTransform, proximal, distal);

        endPosition = GetTransformPosition(jointTransforms.back());
        endRotation = GetTransformRotation(jointTransforms.back());

        // WHITE is the deadman, as for the arm; GREY is left to ur16e_touch_teleop
        // (record / switch module)
        bool isEngaged = isWhiteButtonPressed && isMicroModeActive;

        // Update Touch and manipulator origins when teleoperation engages (rising edge)
        if (isEngaged && !wasEngaged)
        {
            touchOriginPose.position = currentTouchPose.position;
            touchOriginPose.orientation = currentTouchPose.orientation;
            endOriginPosition = endPosition;
            endOriginRotation = endRotation;
            RCLCPP_INFO(node->get_logger(), "Button pressed - updating origin position and rotation");
        }
        
        // Enable manipulator teleoperation while engaged
        if (isEngaged)
        {
            if (hasReceivedMotorStates && hasReceivedMotorMinAngles && hasReceivedMotorMaxAngles)
            {
                // Update Touch orientation delta (in manipulator coordinate frame)
                touchPoseDelta.orientation = (touchToManipulatorRotation * currentTouchPose.orientation) * (touchToManipulatorRotation * touchOriginPose.orientation).inverse();

                RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Touch orientation delta: %s", QuaternionToString(touchPoseDelta.orientation).c_str());

                manipulatorOrientation = EigenRotationMatrixToTF2Quaternion(endRotation);

                manipulatorOriginOrientation = EigenRotationMatrixToTF2Quaternion(endOriginRotation);

                manipulatorOrientationDelta = manipulatorOrientation * manipulatorOriginOrientation.inverse();

                // Final movement orientation delta is difference between Touch and manipulator orientation deltas
                movementOrientationDelta = manipulatorOrientation.slerp(
                    touchPoseDelta.orientation * manipulatorOrientationDelta.inverse() * manipulatorOrientation,
                    MANIPULATOR_ROTATION_SCALE_FACTOR
                );

                RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Movement orientation delta: %s", QuaternionToString(movementOrientationDelta).c_str());

                movementRotation = tf2QuaternionToEigenRotationMatrix(movementOrientationDelta);

                Eigen::MatrixXd jacobian = GetJacobian(jointTransforms, proximal, distal);

                jointPositions << proximal.GetPanJointAngle(), proximal.GetTiltJointAngle(), distal.GetPanJointAngle(), distal.GetTiltJointAngle();

                // Desired pose: origin + scaled-down stylus movement, turned into the
                // manipulator frame by the same rotation as the stylus orientation
                tf2::Vector3 touchDelta = tf2::quatRotate(
                    touchToManipulatorRotation, currentTouchPose.position - touchOriginPose.position);

                Eigen::Vector3d desiredPos = endOriginPosition
                    + MICRO_POSITION_SCALE * Eigen::Vector3d(touchDelta.x(), touchDelta.y(), touchDelta.z());

                Eigen::Matrix3d desiredRot = movementRotation;

                Eigen::VectorXd poseDelta = GetPoseDelta(endPosition, desiredPos, endRotation, desiredRot);

                // Weight the position rows (m) up to the scale of the angle rows (rad)
                Eigen::VectorXd taskWeights(6);
                taskWeights << Eigen::Vector3d::Constant(1.0 / POSITION_WEIGHT_LENGTH), Eigen::Vector3d::Ones();
                Eigen::MatrixXd weightedJacobian = taskWeights.asDiagonal() * jacobian;
                Eigen::VectorXd weightedPoseDelta = taskWeights.asDiagonal() * poseDelta;

                if (weightedPoseDelta.norm() > JOINT_UPDATE_MAX_VELOCITY)
                {
                    weightedPoseDelta = CapVectorMagnitude(weightedPoseDelta, JOINT_UPDATE_MAX_VELOCITY).eval();
                }

                Eigen::MatrixXd inverseJacobian = GetInverseJacobianDamped(weightedJacobian, jointPositions, minJointAngles, maxJointAngles, LEAST_SQUARES_DAMPING_FACTOR);

                Eigen::Vector4d jointDeltas = inverseJacobian * weightedPoseDelta;

                RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                    "Tip target offset (%.2f, %.2f, %.2f) mm, tip offset (%.2f, %.2f, %.2f) mm",
                    1e3 * (desiredPos - endOriginPosition).x(), 1e3 * (desiredPos - endOriginPosition).y(),
                    1e3 * (desiredPos - endOriginPosition).z(), 1e3 * (endPosition - endOriginPosition).x(),
                    1e3 * (endPosition - endOriginPosition).y(), 1e3 * (endPosition - endOriginPosition).z());

                RCLCPP_INFO_THROTTLE(
                    node->get_logger(),
                    *node->get_clock(),
                    100,
                    "JOINT DELTAS: %.6f %.6f %.6f %.6f",
                    jointDeltas(0),
                    jointDeltas(1),
                    jointDeltas(2),
                    jointDeltas(3)
                );

                // Apply each joint delta only if every motor stays inside its limits. The
                // motors follow from the joint angles through the tendon model (a joint
                // delta in rad was being added to the motor angles in degrees, so the
                // motors moved ~1/1000 of what they should)
                for (int i = 0; i < 4; i++)
                {
                    ManipulatorModule candidateProximal = proximal, candidateDistal = distal;
                    ApplyJointDelta(candidateProximal, candidateDistal, i, jointDeltas(i));
                    if (MotorsWithinLimits(motorReference + GetMotorPositionsFromJointPositions(candidateProximal, candidateDistal),
                                           motorMinAngles, motorMaxAngles))
                    {
                        proximal = candidateProximal;
                        distal = candidateDistal;
                    }
                }
                motorCommands = motorReference + GetMotorPositionsFromJointPositions(proximal, distal);

                // Clamp motor commands to limits
                motorCommands(0) = clamp(motorCommands(0), motorMinAngles(0), motorMaxAngles(0));
                motorCommands(1) = clamp(motorCommands(1), motorMinAngles(1), motorMaxAngles(1));
                motorCommands(2) = clamp(motorCommands(2), motorMinAngles(2), motorMaxAngles(2));
                motorCommands(3) = clamp(motorCommands(3), motorMinAngles(3), motorMaxAngles(3));

                RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 100, "Sending motor commands: %.6f %.6f %.6f %.6f", motorCommands(0), motorCommands(1), motorCommands(2), motorCommands(3));

                // Whole degrees on the wire: round (assigning the double truncated, so
                // 89.95 went out as 89)
                motor_angles_msg::msg::MotorAngles motorCommandsMsg;
                motorCommandsMsg.proximal_pan_angle = std::lround(motorCommands(0));
                motorCommandsMsg.proximal_tilt_angle = std::lround(motorCommands(1));
                motorCommandsMsg.distal_pan_angle = std::lround(motorCommands(2));
                motorCommandsMsg.distal_tilt_angle = std::lround(motorCommands(3));

                motorCommandsPublisher->publish(motorCommandsMsg);
            }
        }

        // Update previous engaged state for next iteration
        wasEngaged = isEngaged;

        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
