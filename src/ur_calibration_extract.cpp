// ur_calibration_extract: reads the arm's factory kinematic calibration from the UR controller and writes
// it as a kinematics_params_file for ur_description / ur16e_teleop.launch.py.
//
// Why this exists instead of `ros2 launch ur_calibration calibration_correction.launch.py`
// (full story: docs/UR_CALIBRATION_FIX.md):
//
//   The driver in src/Universal_Robots_ROS2_Driver is 3.8.0, built against apt ur_client_library 2.15.2.
//   Its ur_calibration has two bugs, both fixed upstream on the jazzy branch after 3.8.0:
//
//   1. It never connects (upstream 0089621, "#1981"). In client library 2.15.2 only Pipeline::init()
//      opens the primary-interface socket (init -> URProducer::setupProducer -> URStream::connect).
//      calibration_correction.cpp calls pipeline.run() without init(), so it reads from an unopened
//      socket: "Stream is invalid. Connect it before trying to read!", forever.
//   2. Even once connected, the file it writes is wrong (upstream 81df755, "#2030"). Calibration::toYaml()
//      wrote each link rotation with Eigen eulerAngles(0, 1, 2) (R = Rx*Ry*Rz), but URDF <origin rpy> is
//      R = Rz(yaw)*Ry(pitch)*Rx(roll). They differ whenever a nominal quarter-turn is combined with a
//      calibration delta, i.e. on most UR links, so the "calibrated" URDF would not match the controller.
//
//   We don't edit the driver repo (edit scope is this package). Instead vendor/ur_calibration/ holds
//   byte-identical copies of upstream jazzy's calibration.hpp/.cpp (which have fix 2), and this file
//   replaces calibration_correction.cpp + CalibrationConsumer (with fix 1). Once the driver checkout is
//   updated to >= 3.10.0 the stock tool works too; this one stays useful for the checks below.
//
// Added on top of upstream, so a wrong file can't get through silently:
//   - Bounded connect / receive (upstream retries forever and waits forever).
//   - Before writing, rebuilds the chain from the YAML exactly as URDF does and compares it with forward
//     kinematics computed straight from the controller's raw D-H parameters (independent of ur_calibration's
//     chain correction). Refuses to write if they differ by more than CHECK_TOLERANCE.
//   - Optionally compares with the nominal model (nominal_kinematics_file): how far ROS's tool0 has been
//     from the controller's, i.e. the error every run before this file existed carried.
//   - Writes the raw D-H parameters and the check results into the file as comments, so the file is
//     self-describing and can be re-checked offline (dh_file parameter) without the robot.

#include <Eigen/Dense>

#include <chrono>
#include <atomic>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "ur_client_library/comm/parser.h"
#include "ur_client_library/comm/pipeline.h"
#include "ur_client_library/comm/producer.h"
#include "ur_client_library/comm/stream.h"
#include "ur_client_library/primary/primary_parser.h"
#include "ur_client_library/primary/robot_state/kinematics_info.h"

#include "ur_calibration/calibration.hpp"

namespace fs = std::filesystem;

using ur_calibration::Calibration;
using ur_calibration::DHRobot;
using ur_calibration::DHSegment;
using Joints = Eigen::Matrix<double, 6, 1>;

// The YAML chain and the raw D-H chain must agree to this (largest matrix element; metres for the
// translation part). Upstream's own test uses 1e-9 for its (weaker) self-consistency check.
const double CHECK_TOLERANCE = 1e-9;

// Random joint configurations used for the checks
const int CHECK_SAMPLES = 10000;

// Connection: attempts and gap between them, and how long to wait for the KinematicsInfo package once
// connected (the controller sends it right after connecting)
const size_t CONNECT_TRIES = 3;
const std::chrono::seconds CONNECT_RETRY_GAP(2);
const double RECEIVE_TIMEOUT_S = 10.0;

const char* LINK_NAMES[6] = { "shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3" };

// Forward kinematics straight from the controller's D-H parameters (base -> flange), the model the
// controller itself uses: T_i = Rz(theta_i + q_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
Eigen::Matrix4d dhForwardKinematics(const DHRobot& robot, const Joints& q)
{
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  for (size_t i = 0; i < 6; ++i) {
    const DHSegment& s = robot.segments_[i];
    Eigen::Matrix4d rz = Eigen::Matrix4d::Identity();
    rz.topLeftCorner(3, 3) = Eigen::AngleAxisd(s.theta_ + q(i), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    rz(2, 3) = s.d_;
    Eigen::Matrix4d rx = Eigen::Matrix4d::Identity();
    rx.topLeftCorner(3, 3) = Eigen::AngleAxisd(s.alpha_, Eigen::Vector3d::UnitX()).toRotationMatrix();
    rx(0, 3) = s.a_;
    out *= rz * rx;
  }
  return out;
}

// Forward kinematics the way ur_description builds the URDF from a kinematics_params_file:
// each joint origin is <origin xyz rpy>, R = Rz(yaw)*Ry(pitch)*Rx(roll), then rotation about z by q
Eigen::Matrix4d urdfForwardKinematics(const YAML::Node& kinematics, const Joints& q)
{
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  for (size_t i = 0; i < 6; ++i) {
    const YAML::Node link = kinematics[LINK_NAMES[i]];
    Eigen::Matrix4d origin = Eigen::Matrix4d::Identity();
    origin.topLeftCorner(3, 3) = (Eigen::AngleAxisd(link["yaw"].as<double>(), Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(link["pitch"].as<double>(), Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(link["roll"].as<double>(), Eigen::Vector3d::UnitX()))
                                     .toRotationMatrix();
    origin.topRightCorner(3, 1) << link["x"].as<double>(), link["y"].as<double>(), link["z"].as<double>();
    Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
    rotation.topLeftCorner(3, 3) = Eigen::AngleAxisd(q(i), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    out *= origin * rotation;
  }
  return out;
}

// Result of comparing two chains over random joint configurations
struct ChainDifference
{
  double max_element = 0;  // largest |difference| of any matrix element
  double max_pos_mm = 0;
  double rms_pos_mm = 0;
  double max_rot_deg = 0;
};

template <typename FkA, typename FkB>
ChainDifference compareChains(FkA fk_a, FkB fk_b)
{
  // Fixed seed: the same file always gives the same numbers
  std::mt19937 rng(20261103);
  std::uniform_real_distribution<double> joint(-M_PI, M_PI);
  ChainDifference d;
  double sum_sq = 0;
  for (int n = 0; n < CHECK_SAMPLES; ++n) {
    Joints q;
    for (int i = 0; i < 6; ++i)
      q(i) = joint(rng);
    const Eigen::Matrix4d a = fk_a(q);
    const Eigen::Matrix4d b = fk_b(q);
    d.max_element = std::max(d.max_element, (a - b).cwiseAbs().maxCoeff());
    const double pos_mm = 1e3 * (a.topRightCorner(3, 1) - b.topRightCorner(3, 1)).norm();
    d.max_pos_mm = std::max(d.max_pos_mm, pos_mm);
    sum_sq += pos_mm * pos_mm;
    const Eigen::Matrix3d r = a.topLeftCorner(3, 3).transpose() * b.topLeftCorner(3, 3);
    d.max_rot_deg = std::max(d.max_rot_deg, Eigen::AngleAxisd(r).angle() * 180.0 / M_PI);
  }
  d.rms_pos_mm = std::sqrt(sum_sq / CHECK_SAMPLES);
  return d;
}

// Same as upstream ur_calibration::CalibrationConsumer::consume(), but keeps the raw D-H parameters
// (needed for the independent check) instead of only the converted YAML
class KinematicsConsumer : public urcl::comm::IConsumer<urcl::primary_interface::PrimaryPackage>
{
public:
  bool consume(std::shared_ptr<urcl::primary_interface::PrimaryPackage> product) override
  {
    auto kin_info = std::dynamic_pointer_cast<urcl::primary_interface::KinematicsInfo>(product);
    if (kin_info != nullptr && !received_) {
      RCLCPP_INFO(rclcpp::get_logger("ur_calibration_extract"), "%s", product->toString().c_str());
      for (size_t i = 0; i < kin_info->dh_a_.size(); ++i) {
        robot_.segments_.push_back(
            DHSegment(kin_info->dh_d_[i], kin_info->dh_a_[i], kin_info->dh_theta_[i], kin_info->dh_alpha_[i]));
      }
      hash_ = kin_info->toHash();
      received_ = true;
    }
    return true;
  }

  bool received() const
  {
    return received_;
  }

  DHRobot robot_;
  std::string hash_;

private:
  std::atomic<bool> received_{ false };
};

class CalibrationExtract : public rclcpp::Node
{
public:
  CalibrationExtract() : Node("ur_calibration_extract")
  {
    robot_ip_ = declare_parameter<std::string>("robot_ip", "");
    output_filename_ = declare_parameter<std::string>("output_filename", "");
    nominal_file_ = declare_parameter<std::string>("nominal_kinematics_file", "");
    // Offline: read the raw D-H parameters from a file (the "# dh_*" comment lines this tool writes, or a
    // plain YAML with d/a/theta/alpha lists) instead of the robot
    dh_file_ = declare_parameter<std::string>("dh_file", "");
  }

  // Fills robot_ and hash_ from the controller or dh_file. False on failure.
  bool readKinematics()
  {
    if (!dh_file_.empty())
      return readDhFile();
    if (robot_ip_.empty()) {
      RCLCPP_FATAL(get_logger(), "Set robot_ip (or dh_file to re-check a saved calibration offline)");
      return false;
    }

    urcl::comm::URStream<urcl::primary_interface::PrimaryPackage> stream(robot_ip_,
                                                                         urcl::primary_interface::UR_PRIMARY_PORT);
    urcl::primary_interface::PrimaryParser parser;
    urcl::comm::URProducer<urcl::primary_interface::PrimaryPackage> prod(stream, parser);
    KinematicsConsumer consumer;
    urcl::comm::INotifier notifier;
    urcl::comm::Pipeline<urcl::primary_interface::PrimaryPackage> pipeline(prod, &consumer, "Pipeline", notifier);

    // THE FIX for bug 1: in ur_client_library 2.15.2 init() is what opens the socket. Upstream 3.8.0
    // went straight to run(). Throws UrException if the robot can't be reached.
    RCLCPP_INFO(get_logger(), "Connecting to the primary interface at %s:%d", robot_ip_.c_str(),
                urcl::primary_interface::UR_PRIMARY_PORT);
    pipeline.init(CONNECT_TRIES, CONNECT_RETRY_GAP);
    pipeline.run();

    const auto start = std::chrono::steady_clock::now();
    while (!consumer.received()) {
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > RECEIVE_TIMEOUT_S) {
        RCLCPP_FATAL(get_logger(), "Connected, but no kinematics info arrived within %.0f s", RECEIVE_TIMEOUT_S);
        pipeline.stop();
        return false;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    pipeline.stop();
    robot_ = consumer.robot_;
    hash_ = consumer.hash_;
    source_ = "robot " + robot_ip_ + ":" + std::to_string(urcl::primary_interface::UR_PRIMARY_PORT);
    return true;
  }

  // Converts, checks and writes. False if a check fails or the file can't be written.
  bool convertCheckAndWrite()
  {
    if (robot_.segments_.size() != 6) {
      RCLCPP_FATAL(get_logger(), "Expected 6 D-H segments, got %zu", robot_.segments_.size());
      return false;
    }

    // Conversion, unchanged from upstream (vendor/ur_calibration, includes the rpy fix)
    Calibration calibration(robot_);
    calibration.correctChain();
    YAML::Node yaml = calibration.toYaml();
    yaml["kinematics"]["hash"] = hash_;

    // Check: the file, read the way URDF reads it, must reproduce the controller's own D-H model.
    // A 3.8.0 toYaml() file fails this (rotation errors of order the calibration deltas).
    const YAML::Node kin = yaml["kinematics"];
    const ChainDifference self = compareChains([&](const Joints& q) { return urdfForwardKinematics(kin, q); },
                                               [&](const Joints& q) { return dhForwardKinematics(robot_, q); });
    std::ostringstream check;
    check << std::scientific << std::setprecision(2) << "max element " << self.max_element << ", max position "
          << self.max_pos_mm << " mm, max rotation " << self.max_rot_deg << " deg over " << CHECK_SAMPLES
          << " random poses";
    if (self.max_element > CHECK_TOLERANCE) {
      RCLCPP_FATAL(get_logger(), "CHECK FAILED - the YAML does not reproduce the controller's kinematics (%s). "
                                 "Not writing %s.", check.str().c_str(), output_filename_.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Check passed: YAML (URDF convention) == controller D-H model (%s)",
                check.str().c_str());

    // How far the nominal model is from this arm: the kinematic error of every run without this file
    std::string nominal_line;
    if (!nominal_file_.empty()) {
      try {
        const YAML::Node nominal = YAML::LoadFile(nominal_file_)["kinematics"];
        const ChainDifference nom =
            compareChains([&](const Joints& q) { return urdfForwardKinematics(nominal, q); },
                          [&](const Joints& q) { return dhForwardKinematics(robot_, q); });
        std::ostringstream s;
        s << std::fixed << std::setprecision(3) << "flange position max " << nom.max_pos_mm << " mm, RMS "
          << nom.rms_pos_mm << " mm; rotation max " << nom.max_rot_deg << " deg (" << CHECK_SAMPLES
          << " random poses, joints in +-pi)";
        nominal_line = s.str();
        RCLCPP_INFO(get_logger(), "Nominal model (%s) vs this arm: %s", nominal_file_.c_str(), nominal_line.c_str());
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Could not compare with nominal_kinematics_file %s: %s", nominal_file_.c_str(),
                    e.what());
      }
    }

    if (output_filename_.empty()) {
      RCLCPP_FATAL(get_logger(), "Set output_filename");
      return false;
    }
    const fs::path out_path = fs::absolute(output_filename_);
    if (!fs::exists(out_path.parent_path())) {
      RCLCPP_FATAL(get_logger(), "Folder %s does not exist", out_path.parent_path().c_str());
      return false;
    }
    if (fs::exists(out_path))
      RCLCPP_WARN(get_logger(), "%s already exists. Overwriting.", out_path.c_str());

    std::ofstream file(out_path);
    if (!file.is_open()) {
      RCLCPP_FATAL(get_logger(), "Can't write %s", out_path.c_str());
      return false;
    }
    const std::time_t now = std::time(nullptr);
    file << "# UR factory kinematic calibration, for kinematics_params_file:=\n"
         << "# Written by macro_micro_robot ur_calibration_extract (docs/UR_CALIBRATION_FIX.md) on "
         << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") << "\n"
         << "# Source: " << source_ << "\n"
         << "# Check (this file read as URDF vs controller D-H): " << check.str() << "\n";
    if (!nominal_line.empty())
      file << "# Nominal model vs this arm: " << nominal_line << "\n";
    file << "# Raw controller D-H parameters (re-check offline with dh_file:=<this file>):\n"
         << std::setprecision(17);
    const char* names[4] = { "d", "a", "theta", "alpha" };
    for (int p = 0; p < 4; ++p) {
      file << "# dh_" << names[p] << ": [";
      for (size_t i = 0; i < 6; ++i) {
        const DHSegment& s = robot_.segments_[i];
        const double v[4] = { s.d_, s.a_, s.theta_, s.alpha_ };
        file << (i ? ", " : "") << v[p];
      }
      file << "]\n";
    }
    file << yaml << "\n";
    RCLCPP_INFO(get_logger(), "Wrote %s (hash %s)", out_path.c_str(), hash_.c_str());
    return true;
  }

private:
  bool readDhFile()
  {
    // Accept both a plain YAML (d: [...]) and this tool's own output (# dh_d: [...] comment lines)
    std::ifstream in(dh_file_);
    if (!in.is_open()) {
      RCLCPP_FATAL(get_logger(), "Can't read dh_file %s", dh_file_.c_str());
      return false;
    }
    std::string text, line;
    while (std::getline(in, line)) {
      if (line.rfind("# dh_", 0) == 0)
        text += line.substr(5) + "\n";
    }
    YAML::Node dh = text.empty() ? YAML::LoadFile(dh_file_) : YAML::Load(text);
    try {
      for (size_t i = 0; i < 6; ++i) {
        robot_.segments_.push_back(DHSegment(dh["d"][i].as<double>(), dh["a"][i].as<double>(),
                                             dh["theta"][i].as<double>(), dh["alpha"][i].as<double>()));
      }
      // Our own output has the hash in it; a plain D-H file may have "hash:"
      YAML::Node own = YAML::LoadFile(dh_file_);
      if (own["kinematics"] && own["kinematics"]["hash"])
        hash_ = own["kinematics"]["hash"].as<std::string>();
      else if (dh["hash"])
        hash_ = dh["hash"].as<std::string>();
      else
        hash_ = "unknown";
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "dh_file %s needs 6 values each for d, a, theta, alpha: %s", dh_file_.c_str(),
                   e.what());
      return false;
    }
    source_ = "dh_file " + fs::absolute(dh_file_).string();
    return true;
  }

  std::string robot_ip_, output_filename_, nominal_file_, dh_file_;
  std::string source_;
  DHRobot robot_;
  std::string hash_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  int status = 1;
  try {
    auto node = std::make_shared<CalibrationExtract>();
    if (node->readKinematics() && node->convertCheckAndWrite())
      status = 0;
  } catch (const std::exception& e) {
    // UrException from init() lands here: robot unreachable / primary interface disabled
    RCLCPP_FATAL(rclcpp::get_logger("ur_calibration_extract"), "%s", e.what());
  }
  rclcpp::shutdown();
  return status;
}
