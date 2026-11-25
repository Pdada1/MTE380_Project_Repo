#include "controller.h"
#include "LinearFit.h" //SimpleRegression library
#include "ParabolicFit.h"

namespace autorotate {

#define POS_SAMPLES (int)(1) //1 sample
#define VEL_SAMPLES (int)(4) //4 samples
#define ACC_SAMPLES (int)(12) //12 samples

static const double DEG2RAD = PI / 180.0;

float int_ex = 0;
float int_ey = 0;

//Rotates a camera-frame (x,y) pair into the platform frame, i.e. by -theta0.
//(The platform x axis points along (cos(theta0), sin(theta0)) in the camera frame.)
static void rotate_to_platform(float theta0_deg, float x, float y, float* xp, float* yp) {
    float th = theta0_deg * (float)DEG2RAD;
    float c = cos(th);
    float s = sin(th);
    *xp =  c*x + s*y;
    *yp = -s*x + c*y;
}

float e_av(BallData* ball_data, char axis) {
    float* x;
    float* xr;
    switch (axis) {
        case 'x':
            x = ball_data->x;
            xr = ball_data->xr;
            break;
        case 'y':
            x = ball_data->y;
            xr = ball_data->yr;
            break;
        default:
            return 0;
    }
    
    //Average position over latest POS_SAMPLES
    float sum = 0;
    for (int i=CAMERA_SAMPLES-POS_SAMPLES; i<CAMERA_SAMPLES; i++) {
        sum += x[i] - xr[i];
    }
    return sum/POS_SAMPLES;
}

float de_av(BallData* ball_data, char axis) {
    float* x;
    float* xr;
    switch (axis) {
        case 'x':
            x = ball_data->x;
            xr = ball_data->xr;
            break;
        case 'y':
            x = ball_data->y;
            xr = ball_data->yr;
            break;
        default:
            return 0;
    }
    
    //Generate error/time arrays
    float ex[VEL_SAMPLES] = {0};
    float t[VEL_SAMPLES] = {0};
    for (int i=0; i<VEL_SAMPLES; i++) {
        ex[i] = x[CAMERA_SAMPLES-VEL_SAMPLES+i] - xr[CAMERA_SAMPLES-VEL_SAMPLES+i]; // originally subtracted xr here too, but that caused setpoint kick, so now its just x
        t[i] = ball_data->t[CAMERA_SAMPLES-VEL_SAMPLES+i] - ball_data->t[0]; //Use difference to keep numbers from getting large for LinearFit
    }

    //Linear regression to find average velocity
    LinearFit<float, float> lf;
    lf.fit(t, ex, VEL_SAMPLES);

    float slope = lf.slope();

    return slope;
}

float vel_av(BallData* ball_data, char axis) {
    float* x;
    switch (axis) {
        case 'x':
            x = ball_data->x;
            break;
        case 'y':
            x = ball_data->y;
            break;
        default:
            return 0;
    }
    
    //Generate error/time arrays
    float ex[VEL_SAMPLES] = {0};
    float t[VEL_SAMPLES] = {0};
    for (int i=0; i<VEL_SAMPLES; i++) {
        ex[i] = x[CAMERA_SAMPLES-VEL_SAMPLES+i]; // originally subtracted xr here too, but that caused setpoint kick, so now its just x
        t[i] = ball_data->t[CAMERA_SAMPLES-VEL_SAMPLES+i] - ball_data->t[0]; //Use difference to keep numbers from getting large for LinearFit
    }

    //Linear regression to find average velocity
    LinearFit<float, float> lf;
    lf.fit(t, ex, VEL_SAMPLES);

    float slope = lf.slope();

    return slope;
}

float inte(BallData* ball_data, char axis, float theta0_deg) { //Make sure to only call once (for 'x' and then 'y') per new sample, otherwise integral will be inaccurate
    float dt = ball_data->t[CAMERA_SAMPLES-1] - ball_data->t[CAMERA_SAMPLES-2];

    //Latest error in the camera frame, rotated into the platform frame. The integral is accumulated from this already-rotated error.
    float ex = ball_data->x[CAMERA_SAMPLES-1] - ball_data->xr[CAMERA_SAMPLES-1];
    float ey = ball_data->y[CAMERA_SAMPLES-1] - ball_data->yr[CAMERA_SAMPLES-1];
    float ex_p, ey_p;
    rotate_to_platform(theta0_deg, ex, ey, &ex_p, &ey_p);

    switch (axis) {
        case 'x':
            int_ex += ex_p*dt;
            int_ex = constrain(int_ex, -I_LIMIT, I_LIMIT);
            return int_ex;
        case 'y':
            int_ey += ey_p*dt;
            int_ey = constrain(int_ey, -I_LIMIT, I_LIMIT);
            return int_ey;
        default:
            return 0;
    }
}

float acc_av(BallData* ball_data, char axis) {
    float* x;
    switch (axis) {
        case 'x':
            x = ball_data->x;
            break;
        case 'y':
            x = ball_data->y;
            break;
        default:
            return 0;
    }
    
    //Generate error/time arrays
    float pos[ACC_SAMPLES] = {0};
    float t[ACC_SAMPLES] = {0};
    for (int i=0; i<ACC_SAMPLES; i++) {
        pos[i] = x[CAMERA_SAMPLES-ACC_SAMPLES+i]; // originally subtracted xr here too, but that caused setpoint kick, so now its just x
        t[i] = ball_data->t[CAMERA_SAMPLES-ACC_SAMPLES+i] - ball_data->t[0]; //Use difference to keep numbers from getting large for LinearFit
    }

    //Linear regression to find average acceleration
    ParabolicFit<float, float> pf;
    pf.fit(t, pos, ACC_SAMPLES);

    float acc = 2*pf.a();

    return acc;
}

float pid_get_tilt(BallData* ball_data, char axis, float theta0_deg) { //Gets tilt for axis 'x' or 'y'
    if (axis != 'x' && axis != 'y') {
        return 0;
    }

    //Error and velocity in the camera frame
    float ex = e_av(ball_data, 'x');
    float ey = e_av(ball_data, 'y');
    float vx = vel_av(ball_data, 'x');
    float vy = vel_av(ball_data, 'y');

    //Rotate both into the platform frame
    float ex_p, ey_p, vx_p, vy_p;
    rotate_to_platform(theta0_deg, ex, ey, &ex_p, &ey_p);
    rotate_to_platform(theta0_deg, vx, vy, &vx_p, &vy_p);

    float e     = (axis == 'x') ? ex_p : ey_p;
    float d_e   = (axis == 'x') ? vx_p : vy_p;
    float int_e = inte(ball_data, axis, theta0_deg); //Integrates the already-rotated error

    float u = e*KP + int_e*KI + d_e*KD;
    return u;
}

float pid_get_xtilt(BallData* ball_data, float theta0_deg) {
    return pid_get_tilt(ball_data, 'x', theta0_deg);
}

float pid_get_ytilt(BallData* ball_data, float theta0_deg) {
    return pid_get_tilt(ball_data, 'y', theta0_deg);
}

//ROTATION FITTING
//
//All per-sample work (tilt look-back, trig, sqrt, np/nps vectors, rejection flags) is done ONCE, when the sample
//arrives (add_rotation_sample), and cached. fit_rotation_data() then only sums the cached vectors and solves the
//4x4 eigenproblem, instead of redoing ~200 sin + ~330 sqrt + 50 acos on every call.

static const fit_real AZ_C         = (fit_real)5.89; //Rolling-sphere acceleration constant (3/5 g) [m/s^2]
static const fit_real AZ_C2        = AZ_C * AZ_C;
static const fit_real DEG_TO_RAD_F = (fit_real)(PI / 180.0);
static const fit_real RAD_TO_DEG_F = (fit_real)(180.0 / PI);

#define FIT_MAX_SWEEPS 10 //Jacobi converges quadratically (3-4 sweeps typical), so this cap is only a safety net

//Builds the unit vectors for one sample:
//  nps = (-sin(thetax), -sin(thetay), 1)/norm              (from the commanded tilt, degrees)
//  np  = (a x (-ay, ax, 0))/norm, a = (ax, ay, -az)        (from the measured acceleration, m/s^2)
//Returns false if az / np is undefined for this sample.
static bool build_vectors(fit_real ax, fit_real ay, fit_real thetax_deg, fit_real thetay_deg, fit_real nps[3], fit_real np_v[3]) {
    nps[0] = -sin(thetax_deg * DEG_TO_RAD_F);
    nps[1] = -sin(thetay_deg * DEG_TO_RAD_F);
    nps[2] = 1;
    fit_real nps_inv = 1 / sqrt(nps[0]*nps[0] + nps[1]*nps[1] + 1);
    nps[0] *= nps_inv;  nps[1] *= nps_inv;  nps[2] *= nps_inv;

    /* az = (c - sqrt(c^2 - 4(ax^2+ay^2))) / 2   (minus root), written in the numerically stable form */
    fit_real r2   = ax*ax + ay*ay;
    fit_real disc = AZ_C2 - 4*r2;
    if (disc < 0 || r2 < (fit_real)1e-24) {
        return false;                       /* az or np undefined */
    }
    fit_real az = 2 * r2 / (AZ_C + sqrt(disc));

    /* The cross product works out to (az*ax, az*ay, ax^2+ay^2). */
    np_v[0] = az * ax;
    np_v[1] = az * ay;
    np_v[2] = r2;
    fit_real np_inv = 1 / sqrt(np_v[0]*np_v[0] + np_v[1]*np_v[1] + np_v[2]*np_v[2]);
    np_v[0] *= np_inv;  np_v[1] *= np_inv;  np_v[2] *= np_inv;
    return true;
}

//Gross-outlier limit on |(ax,ay)|^2 (limit for the maximum possible tilt), computed once
static fit_real acc_limit_sq() {
    static fit_real a_lim2 = -1;
    if (a_lim2 < 0) {
        fit_real s_max = sin(PLANE_ANGLE_MAX * DEG_TO_RAD_F);
        fit_real q_max = 2 * s_max * s_max;                     // both axes at the cap
        fit_real a_lim = (fit_real)ACC_LIMIT_K * AZ_C * sqrt(q_max) / (1 + q_max);
        a_lim2 = a_lim * a_lim;
    }
    return a_lim2;
}

void add_rotation_sample(RotationData* rd, float t_new, float thetax, float thetay, float ax, float ay, bool off_platform) {
    const int hist_n = FIT_HISTORY_SAMPLES;

    //Shift the tilt history and append the new sample
    for (int i = 0; i < hist_n-1; i++) {
        rd->t[i]      = rd->t[i+1];
        rd->thetax[i] = rd->thetax[i+1];
        rd->thetay[i] = rd->thetay[i+1];
    }
    rd->t[hist_n-1]      = t_new;
    rd->thetax[hist_n-1] = thetax;
    rd->thetay[hist_n-1] = thetay;
    if (rd->num_valid < hist_n) {
        rd->num_valid++;
    }

    //Ring-buffer slot for this sample
    int slot = rd->ring_head;
    rd->ring_head = (slot + 1 >= FIT_SAMPLES) ? 0 : slot + 1;

    //The acceleration at t_new was caused by the tilt commanded CONTROLLER_DELAY earlier: linearly interpolate the
    //tilt history at that time. The history is searched backwards from the newest sample, and nothing is extrapolated:
    //if the history doesn't reach back far enough the sample is marked SAMPLE_NO_HISTORY.
    float t_target = t_new - CONTROLLER_DELAY;
    int first_valid = hist_n - rd->num_valid; //Oldest real (non zero-filled) history sample
    int j = hist_n - 2;
    while (j >= first_valid && rd->t[j] > t_target) {
        j--;
    }
    if (j < first_valid) {
        rd->sample_state[slot] = SAMPLE_NO_HISTORY;
        return;
    }
    float dt = rd->t[j+1] - rd->t[j];
    float frac = (dt > 1e-9f) ? (t_target - rd->t[j]) / dt : 0.0f;
    frac = constrain(frac, 0.0f, 1.0f); //Safety: never extrapolate
    float thx = rd->thetax[j] + (rd->thetax[j+1] - rd->thetax[j]) * frac;
    float thy = rd->thetay[j] + (rd->thetay[j+1] - rd->thetay[j]) * frac;

    //Debugging
    rd->thetax_shifted = thx;
    rd->thetay_shifted = thy;

    //DISCARD CHECK 1: ball was off the platform
    if (off_platform) {
        rd->sample_state[slot] = SAMPLE_OFF_PLATFORM;
        return;
    }

    //DISCARD CHECK 2: gross acceleration outlier
    if ((fit_real)ax*ax + (fit_real)ay*ay > acc_limit_sq()) {
        rd->sample_state[slot] = SAMPLE_ACC_TOO_HIGH;
        return;
    }

    //Build and cache np / nps
    if (!build_vectors(ax, ay, thx, thy, rd->nps_v[slot], rd->np_v[slot])) {
        rd->sample_state[slot] = SAMPLE_UNUSABLE; /* az or np undefined: the fit skips this sample */
        return;
    }
    rd->sample_state[slot] = SAMPLE_GOOD;
}

void read_rotation_data(RotationData* rotation_data, BallData* ball_data, float thetax, float thetay) {
    //Flag this sample if the ball was off the platform (|(x,y)| > PLATFORM_RADIUS_LIMIT) at any of the
    //positions that went into this acceleration estimate (the last ACC_SAMPLES positions)
    bool off_platform = false;
    for (int i=CAMERA_SAMPLES-ACC_SAMPLES; i<CAMERA_SAMPLES; i++) {
        float r2 = ball_data->x[i]*ball_data->x[i] + ball_data->y[i]*ball_data->y[i];
        if (r2 > PLATFORM_RADIUS_LIMIT*PLATFORM_RADIUS_LIMIT) {
            off_platform = true;
            break;
        }
    }

    float ax = acc_av(ball_data, 'x') * (S/(float)1000); //Scale back to m/s^2
    float ay = acc_av(ball_data, 'y') * (S/(float)1000);

    //thetax, thetay: tilt already computed by the PID this sample (don't call the PID again: that would integrate twice)
    add_rotation_sample(rotation_data, ball_data->t[CAMERA_SAMPLES-1], thetax, thetay, ax, ay, off_platform);
}

void fit_rotation_data(RotationData* rotation_data) {
    bool off_platform = false, acc_high = false;
    int n_used = 0;
    for (int i = 0; i < FIT_SAMPLES; i++) {
        switch (rotation_data->sample_state[i]) {
            case SAMPLE_GOOD:         n_used++;            break;
            case SAMPLE_OFF_PLATFORM: off_platform = true; break;
            case SAMPLE_ACC_TOO_HIGH: acc_high     = true; break;
            default: break;
        }
    }

    if (n_used < 2) {
        rotation_data->cand_rms_deg = 0;
        rotation_data->cand_msx = 0;
        rotation_data->cand_msy = 0;
        rotation_data->cand_theta0 = 0;
        rotation_data->fit_status = FIT_NOT_ENOUGH_SAMPLES;
        return;
    }

    FitStatus status = FIT_OK;
    if (off_platform) {
        status = FIT_OFF_PLATFORM;
    } else if (acc_high) {
        status = FIT_ACC_TOO_HIGH;
    }

    fit_real msx, msy, theta0;
    {
        /* ---------- 1. attitude profile matrix B = sum( np * nps^T ) over the cached vectors ---------- */
        fit_real B[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
        n_used = 0;

        for (int i = 0; i < FIT_SAMPLES; i++) {
            if (rotation_data->sample_state[i] != SAMPLE_GOOD) {
                continue;                       /* not ready / rejected: skip sample */
            }
            const fit_real* np_v = rotation_data->np_v[i];
            const fit_real* nps  = rotation_data->nps_v[i];
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    B[j][k] += np_v[j] * nps[k];
                }
            }
            n_used++;
        }

        if (n_used < 2) {
            rotation_data->cand_rms_deg = 0;
            rotation_data->cand_msx = 0;
            rotation_data->cand_msy = 0;
            rotation_data->cand_theta0 = 0;
            rotation_data->fit_status = (status == FIT_OK) ? FIT_DEGENERATE : status;
            return;                             /* not enough usable samples: keep old estimate */
        }

        /* ---------- 2. Davenport K matrix (4x4 symmetric) ---------- */
        fit_real sigma = B[0][0] + B[1][1] + B[2][2];
        fit_real zx = B[1][2] - B[2][1];
        fit_real zy = B[2][0] - B[0][2];
        fit_real zz = B[0][1] - B[1][0];

        fit_real K[4][4];
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                K[j][k] = B[j][k] + B[k][j] - ((j == k) ? sigma : 0);
            }
        }
        K[0][3] = zx;  K[3][0] = zx;
        K[1][3] = zy;  K[3][1] = zy;
        K[2][3] = zz;  K[3][2] = zz;
        K[3][3] = sigma;

        /* ---------- 3. eigenvectors of K by Jacobi rotations ---------- */
        //Convergence is relative to the size of K (an absolute tolerance can be unreachable in float)
        fit_real fro2 = 0;
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                fro2 += K[j][k] * K[j][k];
            }
        }
        const fit_real tol2  = (fit_real)1e-12 * fro2; //Stop when the off-diagonal sum of squares is below this
        const fit_real skip2 = tol2 / 6;               //An element this small needs no rotation

        fit_real V[4][4] = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};

        for (int sweep = 0; sweep < FIT_MAX_SWEEPS; sweep++) {
            fit_real off = 0;
            for (int p = 0; p < 3; p++) {
                for (int q = p + 1; q < 4; q++) {
                    off += K[p][q] * K[p][q];
                }
            }
            if (off <= tol2) {
                break;
            }
            for (int p = 0; p < 3; p++) {
                for (int q = p + 1; q < 4; q++) {
                    if (K[p][q] * K[p][q] <= skip2) {
                        continue;
                    }
                    fit_real th = (K[q][q] - K[p][p]) / (2 * K[p][q]);
                    fit_real tn = ((th >= 0) ? 1 : -1)
                                / (fabs(th) + sqrt(th*th + 1));
                    fit_real cs = 1 / sqrt(tn*tn + 1);
                    fit_real sn = tn * cs;

                    for (int k = 0; k < 4; k++) {          /* K = K * J   */
                        fit_real kp = K[k][p], kq = K[k][q];
                        K[k][p] = cs*kp - sn*kq;
                        K[k][q] = sn*kp + cs*kq;
                    }
                    for (int k = 0; k < 4; k++) {          /* K = J^T * K */
                        fit_real kp = K[p][k], kq = K[q][k];
                        K[p][k] = cs*kp - sn*kq;
                        K[q][k] = sn*kp + cs*kq;
                    }
                    for (int k = 0; k < 4; k++) {          /* V = V * J   */
                        fit_real vp = V[k][p], vq = V[k][q];
                        V[k][p] = cs*vp - sn*vq;
                        V[k][q] = sn*vp + cs*vq;
                    }
                }
            }
        }

        /* eigenvector of the largest eigenvalue = optimal quaternion
        q = (q1, q2, q3, q4), scalar last */
        int imax = 0;
        for (int k = 1; k < 4; k++) {
            if (K[k][k] > K[imax][imax]) {
                imax = k;
            }
        }
        fit_real q1 = V[0][imax], q2 = V[1][imax], q3 = V[2][imax], q4 = V[3][imax];
        fit_real qn = sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
        q1 /= qn;  q2 /= qn;  q3 /= qn;  q4 /= qn;

        /* ---------- 4. rotation matrix entries needed (np = R * nps) ---------- */
        fit_real d = q4*q4 - q1*q1 - q2*q2 - q3*q3;
        fit_real R00 = d + 2*q1*q1;
        fit_real R10 = 2*(q1*q2 - q4*q3);
        fit_real R20 = 2*(q1*q3 + q4*q2);
        fit_real R02 = 2*(q1*q3 - q4*q2);
        fit_real R12 = 2*(q2*q3 + q4*q1);
        fit_real R22 = d + 2*q3*q3;

        /* ---------- 5. back-calculate msx, msy, theta0 ---------- */
        if (!(R22 > (fit_real)1e-9)) {              /* ns_z must be > 0 for ns = (-msx, -msy, 1)/norm */
            rotation_data->cand_rms_deg = 0;
            rotation_data->cand_msx = 0;
            rotation_data->cand_msy = 0;
            rotation_data->cand_theta0 = 0;
            rotation_data->fit_status = (status == FIT_OK) ? FIT_DEGENERATE : status;
            return;
        }

        /* third column of R is ns = (-msx, -msy, 1) / norm */
        msx = -R02 / R22;
        msy = -R12 / R22;

        /* s = (1, 0, msx) / norm */
        fit_real s_len = sqrt(1 + msx*msx);
        fit_real sx = 1 / s_len;
        fit_real sz = msx / s_len;

        /* c = ns x s   (s_y = 0) */
        fit_real cx = R12*sz;                       /* ns_y*s_z - ns_z*0         */
        fit_real cy = R22*sx - R02*sz;              /* ns_z*s_x - ns_x*s_z       */
        fit_real cz = -R12*sx;                      /* ns_x*0   - ns_y*s_x       */

        /* first column of R is x0 = s*cos(theta0) + c*sin(theta0) */
        fit_real x0_dot_s = R00*sx + R20*sz;
        fit_real x0_dot_c = R00*cx + R10*cy + R20*cz;
        theta0 = atan2(x0_dot_c, x0_dot_s) * RAD_TO_DEG_F;

        /* ---------- 6. RMS angle between np_calculated = [x0 y0 ns]*nps and np_measured ---------- */
        //For unit vectors, sum |np - R*nps|^2 = 2*(n - lambda_max), where lambda_max is the largest eigenvalue of K.
        //|np - R*nps| = 2*sin(angle/2) ~ angle, so this is the RMS angle (agrees with the explicit per-sample angle
        //to within ~angle^2/24, i.e. 0.01 % at 3 deg) without a second pass over the samples or any acos().
        fit_real mse = 2 * ((fit_real)n_used - K[imax][imax]) / n_used;
        if (mse < 0) {
            mse = 0;                                //Rounding can push a perfect fit slightly negative
        }
        fit_real rms_deg = sqrt(mse) * RAD_TO_DEG_F;
        rotation_data->rms_deg = rms_deg;

        //Always save candidate fit results even if status is bad
        rotation_data->cand_rms_deg = (float)rms_deg;
        rotation_data->cand_msx = (float)msx;
        rotation_data->cand_msy = (float)msy;
        rotation_data->cand_theta0 = (float)theta0;

        //DISCARD CHECK 3: fit too poor (also rejects NaN)
        if (status == FIT_OK && !(rms_deg <= RMS_LIMIT_DEG)) {
            status = FIT_RMS_TOO_HIGH;
        }
    }

    if (!(fabs(msx) < 1e9 && fabs(msy) < 1e9 && fabs(theta0) <= 360)) { //Also rejects NaN/inf
        rotation_data->fit_status = (status == FIT_OK) ? FIT_DEGENERATE : status;
        return;
    }

    rotation_data->fit_status = status;

    if (status == FIT_OK) {
        rotation_data->rms_deg = (float)rotation_data->cand_rms_deg;
        rotation_data->msx = (float)msx;
        rotation_data->msy = (float)msy;
        rotation_data->theta0 = (float)theta0;
    }
}
} // namespace autorotate
