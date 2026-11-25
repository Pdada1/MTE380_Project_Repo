#include "controller.h"
#include "../alternate_tracking.h"

namespace autorotate {

static BallData ballData;
static RotationData rotationData;
static uint16_t num_samples = 0;

// ================= RAM LOGGING =================

#define LOG_BUFFER_SIZE 200

struct LogEntry {
    uint32_t t_ms;
    uint8_t fit_status;
    uint16_t rms_deg;
    int16_t msx;
    int16_t msy;
    int32_t theta0;
};

LogEntry logBuffer[LOG_BUFFER_SIZE];
uint8_t logCount = 0;

void flushLogBuffer() {
    for (uint8_t i = 0; i < logCount; i++) {
        LogEntry &d = logBuffer[i];
        Serial.print(d.t_ms / 1000.0f, 3);
        Serial.print(' ');
        Serial.print(d.fit_status);
        Serial.print(' ');
        Serial.print(d.rms_deg / 1000.0f, 3);
        Serial.print(' ');
        Serial.print(d.msx / 1000.0f, 3);
        Serial.print(' ');
        Serial.print(d.msy / 1000.0f, 3);
        Serial.print(' ');
        Serial.println(d.theta0 / 1000.0f, 3);
    }
    logCount = 0;
}

void storeLog(const LogEntry &entry) {
    logBuffer[logCount++] = entry;
    if (logCount >= LOG_BUFFER_SIZE) {
        flushLogBuffer();
    }
}

void track(const AlternateFrame& frame, ApplyAlternateTilt apply_tilt) {
    // The main firmware owns serial and provides the newest camera frame.
    for (int i = 0; i < CAMERA_SAMPLES - 1; ++i) {
        ballData.x[i] = ballData.x[i + 1];
        ballData.y[i] = ballData.y[i + 1];
        ballData.xr[i] = ballData.xr[i + 1];
        ballData.yr[i] = ballData.yr[i + 1];
        ballData.t[i] = ballData.t[i + 1];
    }
    const int latest = CAMERA_SAMPLES - 1;
    ballData.x[latest] = frame.x;
    ballData.y[latest] = frame.y;
    ballData.xr[latest] = frame.xr;
    ballData.yr[latest] = frame.yr;
    ballData.t[latest] = frame.seconds;

    unsigned long t0 = micros();
    static float last_t = 0;
    float now = ballData.t[CAMERA_SAMPLES-1];
    if (now - last_t > 0.3f) {            // dropout: buffers hold stale data
        num_samples = 1;                  // wait for CAMERA_SAMPLES fresh samples
    }
    last_t = now;
    num_samples++;

    //Only run controller after ballData buffers have filled
    if (num_samples >= CAMERA_SAMPLES) {
        float thetax = pid_get_xtilt(&ballData, rotationData.theta0);
        float thetay = pid_get_ytilt(&ballData, rotationData.theta0);
        thetax = constrain(thetax, -PLANE_ANGLE_MAX, PLANE_ANGLE_MAX);
        thetay = constrain(thetay, -PLANE_ANGLE_MAX, PLANE_ANGLE_MAX);
        apply_tilt(thetax, thetay);
        
        // unsigned long t0 = micros();
        read_rotation_data(&rotationData, &ballData, thetax, thetay);
        fit_rotation_data(&rotationData);
        // unsigned long fit_us = micros() - t0; //read_rotation_data() + fit_rotation_data()

        // // t, x, y, ax, ay, sin(thetax), sin(thetay)
        // Serial.print(ballData.t[CAMERA_SAMPLES-1], 3);
        // Serial.print(' ');
        // Serial.print(ballData.x[CAMERA_SAMPLES-1], 3);
        // Serial.print(' ');
        // Serial.print(ballData.y[CAMERA_SAMPLES-1], 3);
        // Serial.print(' ');
        // Serial.print(vel_av(&ballData, 'x'), 3);
        // Serial.print(' ');
        // Serial.print(vel_av(&ballData, 'y'), 3);
        // Serial.print(' ');
        // Serial.print(acc_av(&ballData, 'x'), 3);
        // Serial.print(' ');
        // Serial.print(acc_av(&ballData, 'y'), 3);
        // Serial.print(' ');
        // Serial.print(-sin(thetax*PI/180), 3);
        // Serial.print(' ');
        // Serial.print(-sin(thetay*PI/180), 3);
        // Serial.println(' ');

        // float accx = acc_av(&ballData, 'x');
        // float accy = acc_av(&ballData, 'y');
        // //float acc_norm = sqrt(accx*accx + accy*accy);
        // Serial.print(accx * (S/(float)1000), 6); // m/s^2
        // Serial.print(' ');
        // Serial.print(accy * (S/(float)1000), 6);
        // Serial.print(' ');
        // //Shifted thetax and thetay
        // Serial.print(-sin(rotationData.thetax_shifted*PI/180), 3);
        // Serial.print(' ');
        // Serial.print(-sin(rotationData.thetay_shifted*PI/180), 3);
        // Serial.println(' ');

        // Serial.print(ballData.t[CAMERA_SAMPLES-1], 3);              Serial.print(' ');
        // Serial.print(ballData.x[CAMERA_SAMPLES-1], 3);              Serial.print(' ');
        // Serial.print(ballData.y[CAMERA_SAMPLES-1], 3);              Serial.print(' ');
        // Serial.print(-sin(thetax*PI/180), 2);                       Serial.print(' ');
        // Serial.print(-sin(thetay*PI/180), 2);                       Serial.print(' ');
        // Serial.print(acc_av(&ballData, 'x') * (S/(float)1000), 4);  Serial.print(' ');
        // Serial.print(acc_av(&ballData, 'y') * (S/(float)1000), 4);  Serial.print(' ');
        // Serial.print(-sin(rotationData.thetax_shifted*PI/180), 2);  Serial.print(' ');
        // Serial.print(-sin(rotationData.thetay_shifted*PI/180), 2);  Serial.print(' ');
        // Serial.print((int)rotationData.fit_status);                 Serial.print(' ');
        // Serial.print(rotationData.rms_deg, 2);                      Serial.print(' ');
        // Serial.print(rotationData.msx, 4);                          Serial.print(' ');
        // Serial.print(rotationData.msy, 4);                          Serial.print(' ');
        // Serial.print(rotationData.theta0, 2);                       Serial.print(' ');
        // unsigned long fit_us = micros() - t0;
        // Serial.println(fit_us);

        // Save data to RAM
        float accx = acc_av(&ballData, 'x') * (S / (float)1000);
        float accy = acc_av(&ballData, 'y') * (S / (float)1000);
        LogEntry entry;
        entry.t_ms = (uint32_t)lroundf(ballData.t[CAMERA_SAMPLES-1] * 1000.0f);
        entry.fit_status = (uint8_t)rotationData.fit_status;
        entry.rms_deg = (uint16_t)lroundf(rotationData.cand_rms_deg * 1000.0f);
        entry.msx = (int16_t)lroundf(rotationData.cand_msx * 1000.0f);
        entry.msy = (int16_t)lroundf(rotationData.cand_msy * 1000.0f);
        entry.theta0 = (int32_t)lroundf(rotationData.cand_theta0 * 1000.0f);
        storeLog(entry);
    }
}

} // namespace autorotate
