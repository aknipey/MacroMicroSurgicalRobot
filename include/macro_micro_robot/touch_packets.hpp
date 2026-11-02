// UDP packets between touch_server (OpenHaptics, no ROS) and touch_bridge (ROS, no
// OpenHaptics), both on localhost. See src/touch_server.cpp for why they are split.
#pragma once

#include <cstdint>

constexpr uint16_t TOUCH_STATE_PORT = 47101;   // touch_server -> touch_bridge
constexpr uint16_t TOUCH_FORCE_PORT = 47102;   // touch_bridge -> touch_server
constexpr uint32_t TOUCH_STATE_MAGIC = 0x54535431;  // "TST1"
constexpr uint32_t TOUCH_FORCE_MAGIC = 0x54464331;  // "TFC1"

// One per servo tick (1 kHz). Axes and units as omni_state's /phantom/state: mm,
// x right, y away from the operator, z up.
struct TouchStatePacket
{
    uint32_t magic;
    uint32_t sequence;
    double position[3];         // mm
    double orientation[4];      // quaternion x, y, z, w (omni_state's, incl. its offset)
    double velocity[3];         // mm/s, omni_state's 20 Hz filtered estimate
    double thetas[7];           // omni_state's joint angles (rad), index 0 unused
    double force[3];            // force applied this tick, /phantom/state axes (N)
    int32_t grey_button;
    int32_t white_button;
};

constexpr uint32_t TOUCH_SCENE_MAGIC = 0x54534331;  // "TSC1"

// Force to render, in the /phantom/state axes (N).
struct TouchForcePacket
{
    uint32_t magic;
    uint32_t reserved;
    double force[3];
};

// Position-dependent cues, rendered by touch_server every servo tick from the pen's
// current position (a spring computed over ROS is too late and shakes the pen). Along
// `axis` from `origin`, u = (pen - origin) . axis: a wall pushing back up past
// u_min and down past u_max, and a detent at u_zero. Sent to TOUCH_FORCE_PORT;
// dropped after the same timeout as a force. Python: struct "<II3d3d7d" (112 bytes).
struct TouchScenePacket
{
    uint32_t magic;
    uint32_t enabled;           // 0 = no scene
    double axis[3];             // unit, /phantom/state axes
    double origin[3];           // mm
    double u_zero;              // mm along axis: detent centre
    double u_min, u_max;        // mm along axis: walls
    double stiffness;           // N/mm, both walls
    double detent_force;        // N, peak
    double detent_width;        // mm, where the peak is
    double max_force;           // N, cap on the total force
};
static_assert(sizeof(TouchScenePacket) == 112, "keep in step with the Python struct format");
