#include "autotune.h"
#include "camera_modes.h"
#include "controller.h"

float at_rise_desired_ms = 1000.0f;
bool use_autotune_pid = false;
float at_overshoot_max = 0.10f;      // fraction (10%)

void autotune_set_rise_time(float ms) {
    // Ignore NaN/garbage, and 0.0 from an older Python build.
    if (isfinite(ms) && ms >= 50.0f && ms <= 20000.0f) {
        at_rise_desired_ms = ms;
    }
}

void autotune_set_overshoot(float percent) {
    if (isfinite(percent) && percent >= 5.0f && percent <= 50.0f) {
        at_overshoot_max = percent / 100.0f;
    }
}



// ---- Two-phase autotune ----
// Phase 1 (KP): P-only step responses. Scale kp_a by t_r/desired until the
//               10%-90% rise time is within tolerance several runs in a row.
// Phase 2 (KD): seed kd_a, run step responses on kp_a/kd_a, measure overshoot,
//               and raise kd_a until overshoot <= the user's maximum.

#define AT_MIN_STEP       0.35f    // radii; smaller target changes are ignored
#define AT_TIMEOUT_MS     6000
#define AT_SP_EPS         1e-3f

// "Settled" = near target and nearly stopped, held for AT_SETTLE_MS
#define AT_SETTLE_ERR     0.25f     // radii
#define AT_SETTLE_SPEED   0.15f    // radii/s (tune to your camera noise)
#define AT_SETTLE_MS      500

// Phase 1
#define AT_KP_TOL             0.10f   // |t_r/desired - 1| counted as a good run
#define AT_KP_CONVERGED_RUNS  2
#define AT_KP_HEADROOM        1.2f    // one-time kp boost to offset kd's slowdown (1.0 = off)

#define AT_KD_PHASE1   0.5f    // kd_a held during the kp phase

// Phase 2
#define AT_KD_SEED            1.0f    // Inital value of kd_a at the beginning of phase 2
#define AT_KD_STEP_MAX        2.0f    // max kd multiplier per run
#define AT_KD_CONVERGED_RUNS  2
#define AT_PEAK_WINDOW_MIN    500     // ms to keep watching for the overshoot peak
#define AT_PEAK_WINDOW_MAX    4000

enum AutotunePhase : uint8_t { AT_PHASE_KP, AT_PHASE_KD, AT_PHASE_DONE };
enum AutotuneState : uint8_t { AT_SETTLING, AT_ARMED, AT_MEASURING };

static AutotunePhase at_phase = AT_PHASE_KP;
static AutotuneState at_state = AT_SETTLING;
static bool at_was_active = false;
static uint8_t at_good_runs = 0;
static float at_e0x, at_e0y, at_e0_sq, at_s_min;
static uint32_t at_t_start, at_t10, at_settle_start, at_arrive_ms, at_window_ms;
static bool at_have10, at_arrived, at_settle_timing;

// Go back to the normal (GUI) gains and wait for the ball to come to rest.
static void at_enter_settling() {
    at_state = AT_SETTLING;
    at_settle_timing = false;
    use_autotune_pid = false;
}

// Damping ratio of a 2nd-order system from fractional overshoot (0..1).
static float damping_from_overshoot(float mp) {
    mp = constrain(mp, 0.005f, 0.95f);
    const float l = log(mp);
    return -l / sqrt(PI * PI + l * l);
}

static void at_finalize() {
    at_phase = AT_PHASE_DONE;
    use_autotune_pid = true;             // keep holding with the tuned gains
    Serial.print(F("Autotune DONE: kp_a="));  Serial.print(autotune_get_kp(), 3);
    Serial.print(F("  kd_a="));               Serial.println(autotune_get_kd(), 3);
}

static void at_finish_kp(uint32_t t_r) {
    const float ratio = (float)t_r / at_rise_desired_ms;
    autotune_scale_kp(ratio);

    const bool in_tol = fabs(ratio - 1.0f) <= AT_KP_TOL;
    const bool saturated = autotune_kp_saturated(ratio);
    at_good_runs = (in_tol || saturated) ? at_good_runs + 1 : 0;

    Serial.print(F("KP: t_r="));        Serial.print(t_r);
    Serial.print(F(" ms  desired="));   Serial.print(at_rise_desired_ms, 0);
    Serial.print(F("  ratio="));        Serial.print(ratio, 3);
    Serial.print(F("  kp_a="));         Serial.print(autotune_get_kp(), 3);
    Serial.print(F("  good_runs="));    Serial.println(at_good_runs);
    if (saturated && !in_tol) Serial.println(F("KP: kp_a limit reached, can't get closer"));

    if (at_good_runs >= AT_KP_CONVERGED_RUNS) {
        autotune_scale_kp(AT_KP_HEADROOM);
        autotune_set_kd(AT_KD_SEED);
        at_phase = AT_PHASE_KD;
        at_good_runs = 0;
        Serial.print(F("KP converged (with headroom): kp_a="));
        Serial.print(autotune_get_kp(), 3);
        Serial.print(F(". Phase 2 (kd), kd_a="));
        Serial.println(autotune_get_kd(), 3);
    }
    at_enter_settling();
}

static void at_finish_kd(float mp) {
    Serial.print(F("KD: overshoot="));   Serial.print(mp * 100.0f, 1);
    Serial.print(F("%  max="));          Serial.print(at_overshoot_max * 100.0f, 1);
    Serial.print(F("%  kd_a="));         Serial.println(autotune_get_kd(), 3);

    if (mp <= at_overshoot_max) {
        if (++at_good_runs >= AT_KD_CONVERGED_RUNS) { at_finalize(); return; }
        Serial.println(F("KD: within limit, one more confirming run"));
    } else {
        at_good_runs = 0;
        if (autotune_kd_at_max()) {
            Serial.println(F("KD: limit reached and still overshooting, stopping"));
            at_finalize();
            return;
        }
        const float ratio = constrain(damping_from_overshoot(at_overshoot_max) /
                                      damping_from_overshoot(mp), 1.0f, AT_KD_STEP_MAX);
        autotune_set_kd(autotune_get_kd() * ratio);
        Serial.print(F("KD: raising kd_a to "));  Serial.println(autotune_get_kd(), 3);
    }
    at_enter_settling();
}

bool autotune_routine(BallData* d) {
    const bool active = (d->status == PacketStatus::Autotune_MM);

    if (!active) {                       // left the mode: reset, normal gains
        at_was_active = false;
        at_enter_settling();
        return false;
    }
    if (!at_was_active) {                // just entered: restart at phase 1
        at_was_active = true;
        at_phase = AT_PHASE_KP;
        autotune_set_kd(AT_KD_PHASE1);   // was autotune_set_kd(0)
        at_good_runs = 0;
        autotune_set_kd(0);              // kp_a is kept from the last session
        at_enter_settling();
        Serial.println(F("Autotune: phase 1 (kp), waiting for the ball to settle"));
        return false;
    }
    if (at_phase == AT_PHASE_DONE) return true;

    const int n = CAMERA_SAMPLES - 1;
    const uint32_t now = d->frame_ms[n];
    const float ex = d->x[n] - d->xr[n];
    const float ey = d->y[n] - d->yr[n];
    const bool target_changed = fabs(d->xr[n] - d->xr[n - 1]) > AT_SP_EPS ||
                                fabs(d->yr[n] - d->yr[n - 1]) > AT_SP_EPS;

    // Normalized step size
    const float e0_sq = ex * ex + ey * ey;

    switch (at_state) {
        case AT_SETTLING: {
            if (target_changed) {
                at_settle_timing = false;
                Serial.println(F("Autotune: still settling, target change ignored"));
            }
            const float err = sqrt(ex * ex + ey * ey);
            const float vx = de_av(d, 'x'), vy = de_av(d, 'y');
            const float speed = sqrt(vx * vx + vy * vy);

            if (err < AT_SETTLE_ERR && speed < AT_SETTLE_SPEED) {
                if (!at_settle_timing) {
                    at_settle_timing = true;
                    at_settle_start = now;
                } else if (now - at_settle_start >= AT_SETTLE_MS) {
                    at_state = AT_ARMED;
                    Serial.println(F("Autotune: settled, click a new target"));
                }
            } else {
                at_settle_timing = false;
            }
            break;
        }

        case AT_ARMED: {
            if (!target_changed) break;
            if (e0_sq < AT_MIN_STEP * AT_MIN_STEP) {
                Serial.println(F("Autotune: step too small, pick a farther target"));
                at_enter_settling();
                break;
            }

            at_e0x = ex; at_e0y = ey; at_e0_sq = e0_sq;
            at_t_start = now;
            at_have10 = false;
            at_arrived = false;
            at_s_min = 1.0f;
            at_state = AT_MEASURING;
            use_autotune_pid = true;     // step response runs on kp_a / kd_a
            Serial.print(F("Current PID parameters: kp_a = "));
            Serial.print((double)autotune_get_kp(), 3);
            Serial.print(F("  kd_a = "));
            Serial.print((double)autotune_get_kd(), 3);
            Serial.print(F("  kdi_a = "));
            Serial.print((double)autotune_get_ki(), 3);
            
            Serial.print(F("  Autotune: step of "));
            Serial.print(sqrt(e0_sq), 3);
            Serial.println(F(" radii, measuring"));
            break;
        }

        case AT_MEASURING: {
            if (target_changed) {        // ball isn't at rest; this run is invalid
                Serial.println(F("Autotune: target changed mid-run, aborted"));
                at_enter_settling();
                break;
            }
            const float s = (ex * at_e0x + ey * at_e0y) / at_e0_sq;   // 1 -> 0 (<0 = overshoot)
            if (s < at_s_min) 
                at_s_min = s;

            if (at_phase == AT_PHASE_KP) {
                if (!at_have10 && s <= 0.9f) { 
                    at_t10 = now; 
                    at_have10 = true; 
                }
                if (at_have10 && s <= 0.1f) {
                    uint32_t t_r = now - at_t10;
                    if (t_r < 1) t_r = 1;    // shield from t_r being zero or negative
                    at_finish_kp(t_r);
                } else if (now - at_t_start > AT_TIMEOUT_MS) {
                    Serial.println(F("Autotune: timed out. Increasing kp_a"));
                    // FIXME only increase kp if the ball is confirmed not lost
                    autotune_scale_kp(1.25f);
                    at_good_runs = 0;
                    at_enter_settling();
                }
            } else {                         // AT_PHASE_KD
                if (!at_arrived && s <= 0.1f) {
                    at_arrived = true;
                    at_arrive_ms = now;
                    // Watch for the peak for ~2x the time it took to arrive.
                    at_window_ms = constrain((uint32_t)(2 * (now - at_t_start)),
                                             (uint32_t)AT_PEAK_WINDOW_MIN,
                                             (uint32_t)AT_PEAK_WINDOW_MAX);
                }
                if (at_arrived) {
                    if (now - at_arrive_ms >= at_window_ms) {
                        at_finish_kd(max(0.0f, -at_s_min));
                    }
                } else if (now - at_t_start > AT_TIMEOUT_MS) {
                    Serial.println(F("Autotune: timed out, run ignored (kd unchanged)"));
                    at_enter_settling();
                }
            }
            break;
        }
    }
    return use_autotune_pid;
}