#ifndef ALTERNATE_TRACKING_H
#define ALTERNATE_TRACKING_H

#include <stdint.h>

// A decoded camera frame shared by the mode-specific controllers.
struct AlternateFrame {
    float x, y, xr, yr;
    float seconds;
    uint32_t timestamp_ms;
};
using ApplyAlternateTilt = void (*)(float, float);

struct BallData;

namespace autorotate {
    void track(const AlternateFrame& frame, ApplyAlternateTilt apply_tilt);
}
namespace autotilt {
    void track(const AlternateFrame& frame, ApplyAlternateTilt apply_tilt);
}
namespace autolevel {
    void track(BallData& ball_data, ApplyAlternateTilt apply_tilt);
    void reset();
}

#endif
