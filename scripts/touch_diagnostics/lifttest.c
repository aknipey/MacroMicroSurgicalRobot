/* Lift test: with the pen resting free (out of the inkwell, nobody touching it), ramp an
   upward force (device +y) and report how far the pen rises.

   usage: lifttest <demo|driver> [force N]
     demo   = HelloHapticDevice order: schedule callback, enable forces, start scheduler
     driver = omni_state order: enable forces, start scheduler, omni_state's
              HHD_Auto_Calibration, then schedule a callback with omni_state's frame
              calls (calibration check, transform, joint + gimbal angles, force, buttons)
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduError.h>

static int driver_mode;
static volatile double force_y = 0.0;
static volatile double pos[3];
static volatile int ticks, errors;
static volatile HDerror last_error;
static HDenum calibration_style;

HDCallbackCode HDCALLBACK cb(void *data)
{
    HDErrorInfo e;
    hduVector3Dd p, f(0, force_y, 0);
    if (driver_mode) {
        /* the same calls, in the same order, as omni_state_callback */
        if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
            hdUpdateCalibration(calibration_style);
        hdBeginFrame(hdGetCurrentDevice());
        hduMatrix transform;
        hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
        hduVector3Dd joints, gimbal;
        hdGetDoublev(HD_CURRENT_JOINT_ANGLES, joints);
        hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal);
        p = hduVector3Dd(transform[3][0], transform[3][1], transform[3][2]);
        hdSetDoublev(HD_CURRENT_FORCE, f);
        int buttons = 0;
        hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
        hdEndFrame(hdGetCurrentDevice());
    } else {
        /* as HelloHapticDevice's gravityWellCallback */
        hdBeginFrame(hdGetCurrentDevice());
        hdGetDoublev(HD_CURRENT_POSITION, p);
        hdSetDoublev(HD_CURRENT_FORCE, f);
        hdEndFrame(hdGetCurrentDevice());
    }
    if (HD_DEVICE_ERROR(e = hdGetError())) { errors++; last_error = e.errorCode; }
    for (int i = 0; i < 3; i++) pos[i] = p[i];
    ticks++;
    return HD_CALLBACK_CONTINUE;
}

/* omni_state's HHD_Auto_Calibration, without the ROS logging */
static void auto_calibration(void)
{
    int styles;
    hdGetIntegerv(HD_CALIBRATION_STYLE, &styles);
    if (styles & HD_CALIBRATION_ENCODER_RESET) calibration_style = HD_CALIBRATION_ENCODER_RESET;
    if (styles & HD_CALIBRATION_INKWELL) calibration_style = HD_CALIBRATION_INKWELL;
    if (styles & HD_CALIBRATION_AUTO) calibration_style = HD_CALIBRATION_AUTO;
    printf("calibration styles %d -> using %d, status 0x%x\n", styles, calibration_style, hdCheckCalibration());
    if (calibration_style == HD_CALIBRATION_ENCODER_RESET) {
        do hdUpdateCalibration(calibration_style); while (hdCheckCalibration() != HD_CALIBRATION_OK);
    }
    for (int i = 0; i < 5 && hdCheckCalibration() != HD_CALIBRATION_OK; i++) {
        usleep(1000000);
        if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) hdUpdateCalibration(calibration_style);
    }
    printf("calibration status after: 0x%x (0x5000 = OK)\n", hdCheckCalibration());
}

int main(int argc, char **argv)
{
    driver_mode = argc > 1 && !strcmp(argv[1], "driver");
    double peak = argc > 2 ? atof(argv[2]) : 1.0;
    HDErrorInfo e;
    HHD h = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(e = hdGetError())) { hduPrintError(stderr, &e, "init"); return 1; }
    printf("%s, %s order, %.2f N up\n", hdGetString(HD_DEVICE_MODEL_TYPE), driver_mode ? "omni_state" : "demo", peak);
    HDSchedulerHandle s;
    if (driver_mode) {
        hdEnable(HD_FORCE_OUTPUT);
        hdStartScheduler();
        auto_calibration();
        s = hdScheduleAsynchronous(cb, 0, HD_MAX_SCHEDULER_PRIORITY);
    } else {
        s = hdScheduleAsynchronous(cb, 0, HD_MAX_SCHEDULER_PRIORITY);
        hdEnable(HD_FORCE_OUTPUT);
        hdStartScheduler();
    }
    if (HD_DEVICE_ERROR(e = hdGetError())) hduPrintError(stderr, &e, "start");
    printf("enabled: FORCE_OUTPUT %d MAX_FORCE_CLAMPING %d FORCE_RAMPING %d SOFTWARE_FORCE_LIMIT %d ONE_FRAME_LIMIT %d\n",
           hdIsEnabled(HD_FORCE_OUTPUT), hdIsEnabled(HD_MAX_FORCE_CLAMPING), hdIsEnabled(HD_FORCE_RAMPING),
           hdIsEnabled(HD_SOFTWARE_FORCE_LIMIT), hdIsEnabled(HD_ONE_FRAME_LIMIT));
    usleep((useconds_t)(1e6 * (argc > 3 ? atof(argv[3]) : 0.5)));
    double start[3] = {pos[0], pos[1], pos[2]}, top = start[1];
    for (int i = 0; i <= 150; i++) {       /* 0.3 s ramp up, 1.2 s hold: 10 ms steps */
        force_y = peak * (i < 30 ? i / 30.0 : 1.0);
        if (pos[1] > top) top = pos[1];
        usleep(10000);
    }
    double end[3] = {pos[0], pos[1], pos[2]};
    for (int i = 30; i >= 0; i--) { force_y = peak * i / 30.0; usleep(10000); }
    usleep(500000);
    printf("pen start (%.1f %.1f %.1f) mm, end of push (%.1f %.1f %.1f) mm, highest y %.1f\n",
           start[0], start[1], start[2], end[0], end[1], end[2], top);
    printf("RESULT: rose %.1f mm   (ticks %d, HD errors %d, last 0x%x, forces %s)\n",
           top - start[1], ticks, errors, last_error, hdIsEnabled(HD_FORCE_OUTPUT) ? "enabled" : "DISABLED");
    hdStopScheduler(); hdUnschedule(s); hdDisableDevice(h);
    return 0;
}
