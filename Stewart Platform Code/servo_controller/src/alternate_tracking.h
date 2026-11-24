#ifndef ALTERNATE_TRACKING_H
#define ALTERNATE_TRACKING_H

#include <stdint.h>

// A decoded tracking frame shared by both archived controllers.
struct AlternateFrame {
    float x, y, xr, yr;
    float seconds;
};
using ApplyAlternateTilt = void (*)(float, float);

namespace autorotate {
    void track(const AlternateFrame& frame, ApplyAlternateTilt apply_tilt);
}
namespace autotilt {
    void track(const AlternateFrame& frame, ApplyAlternateTilt apply_tilt);
}

#endif
