// Touch (OpenHaptics) servo loop in a process of its own, with no ROS libraries.
//
// OpenHaptics stops rendering forces - silently: no HD error, the 1 kHz loop runs and
// HD_FORCE_OUTPUT reads enabled - in any process that has liblttng-ust or libnuma
// loaded, and every ROS 2 node loads both (rcl -> tracetools -> lttng-ust -> numa).
// So omni_state's position and buttons work but no force ever reaches the motors.
// This program does the device side exactly as omni_state's servo callback does (same
// axes, units, filters, calibration), and touch_bridge does the ROS side; they talk
// over UDP on localhost. Do not link this executable against anything from ROS.
//
// Each servo tick sends a TouchStatePacket to 127.0.0.1:STATE_PORT. Forces arrive as
// TouchForcePacket on FORCE_PORT, in the /phantom/state axes (N), and are applied as
// they are (omni_state's axis swap). Position-dependent cues (RCM walls and detent)
// arrive as a TouchScenePacket and are rendered here, every tick, from the current
// position: computed in the teleop over ROS, they reached the pen tens of ms late and
// shook it. A force or scene older than FORCE_TIMEOUT is dropped (omni_state kept the
// last force for ever). The total is capped and slew-limited.
//
// omni_state also subtracted 0.001 N per mm/s of its 20 Hz-filtered velocity from every
// force. On the real Touch that "damping" shook the pen at ~25 Hz (the filter's lag
// turns it into a push along the motion there), 2 N while the cue asked for 0.3 N, so it
// is left out: whoever sends the force adds any damping it wants.

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#include <HDU/hduVector.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <mutex>

#include "macro_micro_robot/touch_packets.hpp"

namespace
{

constexpr double FORCE_TIMEOUT = 0.25;      // s without a force / scene packet -> dropped
constexpr double MAX_FORCE = 2.0;           // N, hard cap on everything rendered
constexpr double MAX_STEP = 0.2;            // N per servo tick (200 N/s)

struct ServoState
{
    hduVector3Dd position, velocity;
    hduVector3Dd inp_vel1, inp_vel2, inp_vel3, out_vel1, out_vel2, out_vel3;
    hduVector3Dd pos_hist1, pos_hist2;
};

ServoState servo;
int state_socket = -1;
sockaddr_in state_address{};
uint32_t sequence = 0;
HDenum calibration_style = 0;

std::mutex force_mutex;
double force[3] = {0, 0, 0};            // /phantom/state axes, N
double force_time = -1e9;               // monotonic s of the last force packet
TouchScenePacket scene{};
double scene_time = -1e9;               // monotonic s of the last scene packet
std::atomic<bool> running{true};

double monotonic()
{
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + 1e-9 * t.tv_nsec;
}

// Force (N) along scene.axis at pen position p (mm): one-sided walls past u_min / u_max
// and a detent at u_zero (slope of a Gaussian well, peak detent_force at
// +-detent_width, ~0 past 4 widths)
double render_scene(const TouchScenePacket &s, const hduVector3Dd &p)
{
    double u = 0;
    for (int i = 0; i < 3; i++)
        u += (p[i] - s.origin[i]) * s.axis[i];
    double along = 0;
    if (u < s.u_min)
        along += s.stiffness * (s.u_min - u);
    if (u > s.u_max)
        along -= s.stiffness * (u - s.u_max);
    if (s.detent_width > 0) {
        const double x = (u - s.u_zero) / s.detent_width;
        along -= s.detent_force * x * std::exp(0.5 * (1.0 - x * x));
    }
    return along;
}

bool scene_valid(const TouchScenePacket &s)
{
    const double values[] = {s.axis[0], s.axis[1], s.axis[2], s.origin[0], s.origin[1], s.origin[2], s.u_zero,
                             s.u_min, s.u_max, s.stiffness, s.detent_force, s.detent_width, s.max_force};
    for (double v : values)
        if (!std::isfinite(v))
            return false;
    const double norm = std::sqrt(s.axis[0] * s.axis[0] + s.axis[1] * s.axis[1] + s.axis[2] * s.axis[2]);
    return std::fabs(norm - 1.0) < 1e-3 && s.stiffness >= 0 && s.stiffness < 2.0 && s.max_force >= 0;
}

HDCallbackCode HDCALLBACK servo_callback(void *)
{
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
        hdUpdateCalibration(calibration_style);
    hdBeginFrame(hdGetCurrentDevice());
    hduMatrix transform;
    hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
    hduVector3Dd joints, gimbal;
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, joints);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal);

    // From here to the force: omni_state_callback, unchanged (units = mm)
    // Position: z inverted and y <-> z
    servo.position = hduVector3Dd(transform[3][0], -transform[3][2], transform[3][1]);
    hduMatrix rotation(transform);
    rotation.getRotationMatrix(rotation);
    hduMatrix rotation_offset(0.0, -1.0, 0.0, 0.0,
                              1.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 1.0);
    rotation_offset.getRotationMatrix(rotation_offset);
    hduQuaternion rot(rotation_offset * rotation);
    // Velocity: 2nd order backward difference, then a 20 Hz low-pass
    hduVector3Dd vel_buff = (servo.position * 3 - 4 * servo.pos_hist1 + servo.pos_hist2) / 0.002;
    servo.velocity = (.2196 * (vel_buff + servo.inp_vel3) + .6588 * (servo.inp_vel1 + servo.inp_vel2)) / 1000.0
                     - (-2.7488 * servo.out_vel1 + 2.5282 * servo.out_vel2 - 0.7776 * servo.out_vel3);
    servo.pos_hist2 = servo.pos_hist1;
    servo.pos_hist1 = servo.position;
    servo.inp_vel3 = servo.inp_vel2;
    servo.inp_vel2 = servo.inp_vel1;
    servo.inp_vel1 = vel_buff;
    servo.out_vel3 = servo.out_vel2;
    servo.out_vel2 = servo.out_vel1;
    servo.out_vel1 = servo.velocity;

    // Force: the latest force packet plus the scene rendered at this position (both held
    // while the receiving thread is writing one), each zero once stale
    static double held[3] = {0, 0, 0};
    static double held_time = -1e9;
    static TouchScenePacket held_scene{};
    static double held_scene_time = -1e9;
    if (force_mutex.try_lock()) {
        std::memcpy(held, force, sizeof(held));
        held_time = force_time;
        held_scene = scene;
        held_scene_time = scene_time;
        force_mutex.unlock();
    }
    const double now = monotonic();
    double total[3] = {0, 0, 0};                                // /phantom/state axes
    if (now - held_time < FORCE_TIMEOUT)
        std::memcpy(total, held, sizeof(total));
    double max_force = MAX_FORCE;
    if (now - held_scene_time < FORCE_TIMEOUT && held_scene.enabled) {
        const double along = render_scene(held_scene, servo.position);
        for (int i = 0; i < 3; i++)
            total[i] += along * held_scene.axis[i];
        max_force = std::min(MAX_FORCE, held_scene.max_force);
    }
    // Cap the size, then limit how fast it may change (a jump can trip the Touch's
    // max-force-impulse check)
    static double applied[3] = {0, 0, 0};
    const double size = std::sqrt(total[0] * total[0] + total[1] * total[1] + total[2] * total[2]);
    double step[3], step_size = 0;
    for (int i = 0; i < 3; i++) {
        if (size > max_force)
            total[i] *= max_force / size;
        step[i] = total[i] - applied[i];
        step_size += step[i] * step[i];
    }
    step_size = std::sqrt(step_size);
    for (int i = 0; i < 3; i++)
        applied[i] += step_size > MAX_STEP ? step[i] * MAX_STEP / step_size : step[i];
    hduVector3Dd feedback(applied[0], applied[2], -applied[1]);    // y <-> z, z inverted: device axes
    hdSetDoublev(HD_CURRENT_FORCE, feedback);

    int buttons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
    hdEndFrame(hdGetCurrentDevice());

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Error during servo callback");
        if (hduIsSchedulerError(&error))
            return HD_CALLBACK_DONE;
    }

    TouchStatePacket packet{};
    packet.magic = TOUCH_STATE_MAGIC;
    packet.sequence = sequence++;
    for (int i = 0; i < 3; i++) {
        packet.position[i] = servo.position[i];
        packet.velocity[i] = servo.velocity[i];
        packet.force[i] = applied[i];
    }
    packet.orientation[0] = rot.v()[0];
    packet.orientation[1] = rot.v()[1];
    packet.orientation[2] = rot.v()[2];
    packet.orientation[3] = rot.s();
    // omni_state's thetas: base, then the three arm joints, then the gimbal
    const double thetas[7] = {0., joints[0], joints[1], joints[2] - joints[1], gimbal[0], gimbal[1], gimbal[2]};
    std::memcpy(packet.thetas, thetas, sizeof(thetas));
    packet.grey_button = (buttons & HD_DEVICE_BUTTON_1) ? 1 : 0;
    packet.white_button = (buttons & HD_DEVICE_BUTTON_2) ? 1 : 0;
    sendto(state_socket, &packet, sizeof(packet), MSG_DONTWAIT,
           reinterpret_cast<sockaddr *>(&state_address), sizeof(state_address));
    return HD_CALLBACK_CONTINUE;
}

// omni_state's HHD_Auto_Calibration
void auto_calibration()
{
    int styles = 0;
    hdGetIntegerv(HD_CALIBRATION_STYLE, &styles);
    if (styles & HD_CALIBRATION_ENCODER_RESET) calibration_style = HD_CALIBRATION_ENCODER_RESET;
    if (styles & HD_CALIBRATION_INKWELL) calibration_style = HD_CALIBRATION_INKWELL;
    if (styles & HD_CALIBRATION_AUTO) calibration_style = HD_CALIBRATION_AUTO;
    if (calibration_style == HD_CALIBRATION_ENCODER_RESET) {
        HDErrorInfo error;
        do {
            hdUpdateCalibration(calibration_style);
            std::printf("Calibrating.. (put stylus in well)\n");
            if (HD_DEVICE_ERROR(error = hdGetError())) {
                hduPrintError(stderr, &error, "Reset encoders reset failed.");
                break;
            }
        } while (hdCheckCalibration() != HD_CALIBRATION_OK);
    }
    while (running && hdCheckCalibration() != HD_CALIBRATION_OK) {
        usleep(1000000);
        if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
            std::printf("Please place the device into the inkwell for calibration\n");
        else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
            hdUpdateCalibration(calibration_style);
    }
}

}  // namespace

int main()
{
    std::signal(SIGINT, [](int) { running = false; });
    std::signal(SIGTERM, [](int) { running = false; });

    state_socket = socket(AF_INET, SOCK_DGRAM, 0);
    state_address.sin_family = AF_INET;
    state_address.sin_port = htons(TOUCH_STATE_PORT);
    state_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    int force_socket = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in force_address{};
    force_address.sin_family = AF_INET;
    force_address.sin_port = htons(TOUCH_FORCE_PORT);
    force_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(force_socket, reinterpret_cast<sockaddr *>(&force_address), sizeof(force_address)) != 0) {
        std::perror("touch_server: cannot listen for forces (is another touch_server running?)");
        return 1;
    }
    timeval receive_timeout{0, 100000};     // wake up to notice Ctrl-C
    setsockopt(force_socket, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout));

    HDErrorInfo error;
    HHD device = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        return 1;
    }
    std::printf("Found %s.\n", hdGetString(HD_DEVICE_MODEL_TYPE));
    // HelloHapticDevice's order: callback, forces on, scheduler
    HDSchedulerHandle callback = hdScheduleAsynchronous(servo_callback, nullptr, HD_MAX_SCHEDULER_PRIORITY);
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        return 1;
    }
    auto_calibration();
    std::printf("touch_server running: state -> udp %d, forces <- udp %d (zeroed after %.2f s without one)\n",
                TOUCH_STATE_PORT, TOUCH_FORCE_PORT, FORCE_TIMEOUT);
    std::fflush(stdout);

    while (running) {
        alignas(8) unsigned char buffer[256];
        ssize_t size = recv(force_socket, buffer, sizeof(buffer), 0);
        uint32_t magic = 0;
        if (size >= 4)
            std::memcpy(&magic, buffer, 4);
        if (magic == TOUCH_FORCE_MAGIC && size == static_cast<ssize_t>(sizeof(TouchForcePacket))) {
            TouchForcePacket packet;
            std::memcpy(&packet, buffer, sizeof(packet));
            bool finite = std::isfinite(packet.force[0]) && std::isfinite(packet.force[1]) && std::isfinite(packet.force[2]);
            std::lock_guard<std::mutex> lock(force_mutex);
            for (int i = 0; i < 3; i++)
                force[i] = finite ? packet.force[i] : 0.0;
            force_time = monotonic();
        } else if (magic == TOUCH_SCENE_MAGIC && size == static_cast<ssize_t>(sizeof(TouchScenePacket))) {
            TouchScenePacket packet;
            std::memcpy(&packet, buffer, sizeof(packet));
            if (!scene_valid(packet))
                packet.enabled = 0;
            std::lock_guard<std::mutex> lock(force_mutex);
            scene = packet;
            scene_time = monotonic();
        }
    }

    {
        std::lock_guard<std::mutex> lock(force_mutex);
        force_time = scene_time = -1e9;     // zero force for the last ticks
    }
    usleep(20000);
    hdStopScheduler();
    hdUnschedule(callback);
    hdDisableDevice(device);
    std::printf("touch_server stopped\n");
    return 0;
}
