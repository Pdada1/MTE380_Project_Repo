#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "camera_servo.h"

#define KP 4
#define KI 0.3
#define KD 3
#define I_LIMIT 1.0

#define CONTROLLER_DELAY_MAX 1.0
#define CONTROLLER_DELAY 0.55
#define SAMPLE_RATE_APPROX 0.045

//1: when the tilt history starts (power-up, or after a camera dropout) assume the platform sat at its last commanded tilt
//(level at power-up) for the whole delay window, so delay-shifted tilts are available from the very first sample.
//0: wait CONTROLLER_DELAY (about 17 samples) for real history before the Wahba fit can start.
#define SEED_TILT_HISTORY 1

//Tilt history kept for looking back CONTROLLER_DELAY (needs to cover at least CONTROLLER_DELAY_MAX)
#define FIT_HISTORY_SAMPLES ((int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX) + 2)

//------------------------------------------------------------
// Wahba fit of msx, msy (theta0 fixed at 0)  --  port of YES_msx_msy_wahba.py
//------------------------------------------------------------
#define WAHBA_AZ_CONST 5.89f      //"az constant" c in az = (c - sqrt(c^2 - 4(ax^2+ay^2)))/2
#define WAHBA_FIT_SAMPLES 20      //"Number of samples" used in each fit. Each costs 32 bytes of RAM.
#define WAHBA_SAMPLE_SPACING 2    //"Sample spacing": only every Nth controller sample is stored for fitting once the buffer is full
#define WAHBA_FILL_SPACING 1      //Spacing used while the buffer is still filling (1 = store every sample so the first fits come sooner)
#define WAHBA_MIN_SAMPLES 6       //First fit as soon as this many samples are stored (<= WAHBA_FIT_SAMPLES); the fit then
                                  //keeps refining as the buffer fills up to WAHBA_FIT_SAMPLES.
#define WAHBA_FIT_INTERVAL 1      //Once the buffer is full, refit after this many newly stored samples (1 = every stored sample, i.e. every other real sample)
#define WAHBA_MAX_ITER 20         //"Max iterations" of the Gauss-Newton solver
#define WAHBA_STEP_TOL 1e-5f      //Converged when both msx and msy change by less than this in an iteration
#define WAHBA_REBUILD_EVERY 50    //Recompute the running sums from scratch every N stored samples (bounds float drift)
#define WAHBA_COMPUTE_STATS 0     //1: also compute exact R^2 and rms with a pass over the samples each fit (slow on this board). 0: rms is approximate, r2 = NaN.

//Feeding the fit into the control
#define WAHBA_USE_IN_CONTROL 1    //1: correct the PID tilt command with the fitted msx, msy. 0: log only.
#define WAHBA_MS_SLEW 0.05f      //Largest change of the applied msx / msy per controller sample (0.002 = about 0.11 deg per sample)

//Converts acc_av() output to the units the az constant is expressed in (same scaling as the old debug print)
#ifndef ACC_SCALE
#define ACC_SCALE (S/(float)1000)
#endif

enum WahbaStatus {
    WAHBA_NOT_READY = 0,     //Not enough samples yet
    WAHBA_OK = 1,            //Converged
    WAHBA_NOT_CONVERGED = 2, //Hit WAHBA_MAX_ITER (result still stored)
    WAHBA_FAILED = 3         //Singular / non-finite, previous msx, msy kept
};

struct WahbaSample { //One (np, nps) vector pair, plus the measured horizontal direction ahat (for R^2)
    float np[3];     //Unit normal derived from the measured acceleration
    float nps[3];    //Unit plane normal derived from the (delay-shifted) commanded tilt: (-sin(thx), -sin(thy), 1)/norm
    float ahat[2];   //Measured acceleration direction (ax, ay)/|(ax, ay)|
};

struct RotationData { //Struct to keep track of tilt history and shifted tilt
    //Tilt history, ordered old -> new, used to look back CONTROLLER_DELAY for each new sample
    float t[FIT_HISTORY_SAMPLES] = {0};      //t of each sample [s]
    float thetax[FIT_HISTORY_SAMPLES] = {0}; //commanded tilt [deg]
    float thetay[FIT_HISTORY_SAMPLES] = {0};
    int num_valid = 0; //Number of real samples in the history arrays (they start out zero-filled). Set to 0 to force a refill.

    float msx = 0;
    float msy = 0;
    float theta0 = 0;
    float ms_applied_x = 0; //msx / msy actually used to correct the tilt command: the fit, clamped and slew-limited so the command never steps
    float ms_applied_y = 0;
    float thetax_shifted = 0; //Delay-compensated tilt of the newest sample [deg]
    float thetay_shifted = 0;
    bool shifted_valid = false; //True once the shifted tilt was interpolated from real history. Cleared by a refill.

    //Wahba fit buffer (slots 0..wahba_count-1 are valid; the oldest is overwritten once full)
    WahbaSample wahba[WAHBA_FIT_SAMPLES];
    int wahba_head = 0;                         //Next slot to write
    int wahba_count = 0;                        //Number of stored samples
    int wahba_spacing_ctr = 0;                  //Counts controller samples for WAHBA_SAMPLE_SPACING
    int wahba_since_fit = WAHBA_FIT_INTERVAL;   //Stored samples since the last fit (starts "due")
    float wahba_B[3][3] = {{0}};                //Running sums over the stored samples: B = sum np * nps^T
    float wahba_C[3][3] = {{0}};                //C = sum nps * nps^T
    int wahba_since_rebuild = 0;

    //Wahba fit results
    WahbaStatus fit_status = WAHBA_NOT_READY;
    float r2 = 0;      //Combined R^2 of ahatx/ahaty over the fitted samples
    float rms = 0;     //RMS of the 3D vector residuals
    int fit_iters = 0; //Solver iterations used
};

//theta0_deg: current estimate of theta0 [deg], used to rotate the error/velocity/integral into the platform frame
float pid_get_xtilt(BallData* ball_data, float theta0_deg);
float pid_get_ytilt(BallData* ball_data, float theta0_deg);

float vel_av(BallData* ball_data, char axis);
float acc_av(BallData* ball_data, char axis);

void update_shifted_tilt(RotationData* rd, float t_new, float thetax, float thetay);

//Call once per new sample after update_shifted_tilt(). Stores an (acceleration, shifted tilt) pair and, once enough
//pairs are stored, refits rd->msx / rd->msy (theta0 stays 0). Returns true if a fit was run on this call.
bool wahba_update(RotationData* rd, BallData* ball_data);

//Corrects a desired tilt (thetax, thetay in deg, from the PID) for the fitted rotation R(msx, msy): the plane normal
//actually produced by a command is R * nps(command), so the command is chosen as nps = R^T * n_desired.
//The msx / msy used are ramped toward the latest converged fit at WAHBA_MS_SLEW per call, and
//held when there is no valid fit, so the correction can never make the command jump. Call once per controller sample.
//Returns true if a non-zero correction was applied.
bool wahba_compensate_tilt(RotationData* rd, float* thetax, float* thetay);

#endif
