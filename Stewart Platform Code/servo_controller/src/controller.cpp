#include "controller.h"
#include "LinearFit.h" //SimpleRegression library
#include "ParabolicFit.h"

#define POS_SAMPLES (int)(1) //1 sample
#define VEL_SAMPLES (int)(4) //4 samples
#define ACC_SAMPLES (int)(12) //12 samples

static const double AZ_CONST = 5.89;        //Rolling-sphere acceleration constant (3/5 g) [m/s^2]
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
void read_rotation_data(RotationData* rotation_data, BallData* ball_data, float thetax, float thetay) {
    //Shift data arrays to make room for new sample
    for (int i=0; i<FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1; i++) {
        rotation_data->ax[i] = rotation_data->ax[i+1];
        rotation_data->ay[i] = rotation_data->ay[i+1];
        rotation_data->thetax[i] = rotation_data->thetax[i+1];
        rotation_data->thetay[i] = rotation_data->thetay[i+1];
        rotation_data->t[i] = rotation_data->t[i+1];
        rotation_data->off_platform[i] = rotation_data->off_platform[i+1];
    }

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

    //Add new sample to end of arrays
    rotation_data->ax[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = acc_av(ball_data, 'x') * (S/(float)1000); //Scale back to m/s^2
    rotation_data->ay[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = acc_av(ball_data, 'y') * (S/(float)1000);
    rotation_data->thetax[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = thetax; //Tilt already computed by the PID this sample (don't call the PID again: that would integrate twice)
    rotation_data->thetay[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = thetay;
    rotation_data->t[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = ball_data->t[CAMERA_SAMPLES-1];
    rotation_data->off_platform[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = off_platform;

    //Count real samples (the arrays start out zero-filled)
    if (rotation_data->num_valid < FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)) {
        rotation_data->num_valid++;
    }
}

//Builds the unit vectors for one sample:
//  nps = (-sin(thetax), -sin(thetay), 1)/norm              (from the commanded tilt, degrees)
//  np  = (a x (-ay, ax, 0))/norm, a = (ax, ay, -az)        (from the measured acceleration, m/s^2)
//Returns false if az / np is undefined for this sample.
static bool build_vectors(double ax, double ay, double thetax_deg, double thetay_deg, double nps[3], double np_v[3]) {
    nps[0] = -sin(thetax_deg * DEG2RAD);
    nps[1] = -sin(thetay_deg * DEG2RAD);
    nps[2] = 1.0;
    double nps_len = sqrt(nps[0]*nps[0] + nps[1]*nps[1] + 1.0);
    nps[0] /= nps_len;  nps[1] /= nps_len;  nps[2] /= nps_len;

    /* az = (c - sqrt(c^2 - 4(ax^2+ay^2))) / 2   (minus root) */
    double r2   = ax*ax + ay*ay;
    double disc = AZ_CONST*AZ_CONST - 4.0*r2;
    if (disc < 0.0 || r2 < 1e-24) {
        return false;                       /* az or np undefined */
    }
    double az = 2.0 * r2 / (AZ_CONST + sqrt(disc));

    /* The cross product works out to (az*ax, az*ay, ax^2+ay^2). */
    np_v[0] = az * ax;
    np_v[1] = az * ay;
    np_v[2] = r2;
    double np_len = sqrt(np_v[0]*np_v[0] + np_v[1]*np_v[1] + np_v[2]*np_v[2]);
    np_v[0] /= np_len;  np_v[1] /= np_len;  np_v[2] /= np_len;
    return true;
}

void fit_rotation_data(RotationData* rotation_data) {
    //The arrays are ordered old -> new. The fit window is the newest FIT_SAMPLES samples; the samples before it
    //are the tilt history needed to look back CONTROLLER_DELAY from the start of the window.
    int buffer_size = FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX);
    int t_start_index = buffer_size - FIT_SAMPLES;               //Oldest sample in the window
    int first_valid_index = buffer_size - rotation_data->num_valid; //Oldest real (non zero-filled) sample

    //Need a full window of real samples, plus real history reaching back CONTROLLER_DELAY before the window start
    if (first_valid_index > t_start_index ||
        rotation_data->t[t_start_index] - CONTROLLER_DELAY < rotation_data->t[first_valid_index]) {
        //Serial.println(F("Not enough samples to fit rotation data!"));
        rotation_data->fit_status = FIT_NOT_ENOUGH_SAMPLES;
        return;
    }

    //Window of FIT_SAMPLES
    float ax[FIT_SAMPLES] = {0};
    float ay[FIT_SAMPLES] = {0};
    for (int i=0; i<FIT_SAMPLES; i++) {
        ax[i] = rotation_data->ax[t_start_index+i];
        ay[i] = rotation_data->ay[t_start_index+i];
    }

    //Shift thetax and thetay by CONTROLLER_DELAY: the acceleration at time T[i] was caused by the tilt commanded at
    //T[i] - CONTROLLER_DELAY. Linearly interpolate the recorded tilt history at those times. For the first samples of
    //the window those times fall before the window start, so the history stored before the window is used (nothing
    //is extrapolated; the check above guarantees the history reaches back far enough).
    float thetax[FIT_SAMPLES] = {0};
    float thetay[FIT_SAMPLES] = {0};
    int j = first_valid_index; //Left index of the bracketing pair; only moves forward since the target times increase
    for (int i=0; i<FIT_SAMPLES; i++) {
        float t_target = rotation_data->t[t_start_index+i] - CONTROLLER_DELAY;

        //Move j to the last index with t[j] <= t_target (keeping j+1 inside the array)
        while (j+1 < buffer_size-1 && rotation_data->t[j+1] <= t_target) {
            j++;
        }

        float dt = rotation_data->t[j+1] - rotation_data->t[j];
        float frac = (dt > 1e-9f) ? (t_target - rotation_data->t[j]) / dt : 0.0f;
        frac = constrain(frac, 0.0f, 1.0f); //Safety: never extrapolate

        thetax[i] = rotation_data->thetax[j] + (rotation_data->thetax[j+1] - rotation_data->thetax[j]) * frac;
        thetay[i] = rotation_data->thetay[j] + (rotation_data->thetay[j+1] - rotation_data->thetay[j]) * frac;
    }

    //Debugging
    rotation_data->thetax_shifted = thetax[FIT_SAMPLES-1];
    rotation_data->thetay_shifted = thetay[FIT_SAMPLES-1];

    // Now we have ax[],ay[],thetax[],thetay[] compensated for CONTROLLER_DELAY, all at the same time instances.

    //DISCARD CHECK 1: ball was off the platform (|(x,y)| > PLATFORM_RADIUS_LIMIT) at any sample in the window
    for (int i=0; i<FIT_SAMPLES; i++) {
        if (rotation_data->off_platform[t_start_index+i]) {
            rotation_data->fit_status = FIT_OFF_PLATFORM;
            return; //Keep old msx, msy, theta0
        }
    }

    //DISCARD CHECK 2: gross acceleration outlier (limit for the maximum possible tilt)
    const double s_max  = sin(PLANE_ANGLE_MAX * DEG2RAD);
    const double q_max  = 2.0 * s_max * s_max;                  // both axes at the cap
    const double a_lim  = ACC_LIMIT_K * AZ_CONST * sqrt(q_max) / (1.0 + q_max);
    const double a_lim2 = a_lim * a_lim;
    for (int i = 0; i < FIT_SAMPLES; i++) {
        if ((double)ax[i]*ax[i] + (double)ay[i]*ay[i] > a_lim2) {
            rotation_data->fit_status = FIT_ACC_TOO_HIGH;
            return; //Keep old msx, msy, theta0
        }
    }

    double msx, msy, theta0;
    {
        /* ---------- 1. attitude profile matrix B = sum( np * nps^T ) ---------- */
        double B[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
        int n_used = 0;

        for (int i = 0; i < FIT_SAMPLES; i++) {
            double nps[3], np_v[3];
            if (!build_vectors(ax[i], ay[i], thetax[i], thetay[i], nps, np_v)) {
                continue;                       /* az or np undefined: skip sample */
            }

            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    B[j][k] += np_v[j] * nps[k];
                }
            }
            n_used++;
        }

        if (n_used < 2) {
            rotation_data->fit_status = FIT_DEGENERATE;
            return;                             /* not enough usable samples: keep old estimate */
        }

        /* ---------- 2. Davenport K matrix (4x4 symmetric) ---------- */
        double sigma = B[0][0] + B[1][1] + B[2][2];
        double zx = B[1][2] - B[2][1];
        double zy = B[2][0] - B[0][2];
        double zz = B[0][1] - B[1][0];

        double K[4][4];
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                K[j][k] = B[j][k] + B[k][j] - ((j == k) ? sigma : 0.0);
            }
        }
        K[0][3] = zx;  K[3][0] = zx;
        K[1][3] = zy;  K[3][1] = zy;
        K[2][3] = zz;  K[3][2] = zz;
        K[3][3] = sigma;

        /* ---------- 3. eigenvectors of K by Jacobi rotations ---------- */
        double V[4][4] = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};

        for (int sweep = 0; sweep < 50; sweep++) {
            double off = 0.0;
            for (int p = 0; p < 3; p++) {
                for (int q = p + 1; q < 4; q++) {
                    off += K[p][q] * K[p][q];
                }
            }
            if (off < 1e-12) {
                break;
            }
            for (int p = 0; p < 3; p++) {
                for (int q = p + 1; q < 4; q++) {
                    if (fabs(K[p][q]) < 1e-12) {
                        continue;
                    }
                    double th = (K[q][q] - K[p][p]) / (2.0 * K[p][q]);
                    double tn = ((th >= 0.0) ? 1.0 : -1.0)
                                / (fabs(th) + sqrt(th*th + 1.0));
                    double cs = 1.0 / sqrt(tn*tn + 1.0);
                    double sn = tn * cs;

                    for (int k = 0; k < 4; k++) {          /* K = K * J   */
                        double kp = K[k][p], kq = K[k][q];
                        K[k][p] = cs*kp - sn*kq;
                        K[k][q] = sn*kp + cs*kq;
                    }
                    for (int k = 0; k < 4; k++) {          /* K = J^T * K */
                        double kp = K[p][k], kq = K[q][k];
                        K[p][k] = cs*kp - sn*kq;
                        K[q][k] = sn*kp + cs*kq;
                    }
                    for (int k = 0; k < 4; k++) {          /* V = V * J   */
                        double vp = V[k][p], vq = V[k][q];
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
        double q1 = V[0][imax], q2 = V[1][imax], q3 = V[2][imax], q4 = V[3][imax];
        double qn = sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
        q1 /= qn;  q2 /= qn;  q3 /= qn;  q4 /= qn;

        /* ---------- 4. rotation matrix entries needed (np = R * nps) ---------- */
        double d = q4*q4 - q1*q1 - q2*q2 - q3*q3;
        double R00 = d + 2.0*q1*q1;
        double R10 = 2.0*(q1*q2 - q4*q3);
        double R20 = 2.0*(q1*q3 + q4*q2);
        double R02 = 2.0*(q1*q3 - q4*q2);
        double R12 = 2.0*(q2*q3 + q4*q1);
        double R22 = d + 2.0*q3*q3;

        /* ---------- 5. back-calculate msx, msy, theta0 ---------- */
        if (!(R22 > 1e-9)) {                        /* ns_z must be > 0 for ns = (-msx, -msy, 1)/norm */
            rotation_data->fit_status = FIT_DEGENERATE;
            return;
        }

        /* third column of R is ns = (-msx, -msy, 1) / norm */
        msx = -R02 / R22;
        msy = -R12 / R22;

        /* s = (1, 0, msx) / norm */
        double s_len = sqrt(1.0 + msx*msx);
        double sx = 1.0 / s_len;
        double sz = msx / s_len;

        /* c = ns x s   (s_y = 0) */
        double cx = R12*sz;                     /* ns_y*s_z - ns_z*0         */
        double cy = R22*sx - R02*sz;            /* ns_z*s_x - ns_x*s_z       */
        double cz = -R12*sx;                    /* ns_x*0   - ns_y*s_x       */

        /* first column of R is x0 = s*cos(theta0) + c*sin(theta0) */
        double x0_dot_s = R00*sx + R20*sz;
        double x0_dot_c = R00*cx + R10*cy + R20*cz;
        double theta0_rad = atan2(x0_dot_c, x0_dot_s);
        theta0 = theta0_rad * (180.0 / PI);

        /* ---------- 6. RMS angle between np_calculated = [x0 y0 ns]*nps and np_measured ---------- */
        double cos_t = cos(theta0_rad);
        double sin_t = sin(theta0_rad);
        double X0[3] = { sx*cos_t + cx*sin_t,  cy*sin_t,  sz*cos_t + cz*sin_t };  /* x0 = s cos + c sin */
        double Y0[3] = { cx*cos_t - sx*sin_t,  cy*cos_t,  cz*cos_t - sz*sin_t };  /* y0 = c cos - s sin */
        double NS[3] = { R02, R12, R22 };                                         /* ns                 */

        double sum_sq = 0.0;
        int n_rms = 0;
        for (int i = 0; i < FIT_SAMPLES; i++) {
            double nps[3], np_v[3];
            if (!build_vectors(ax[i], ay[i], thetax[i], thetay[i], nps, np_v)) {
                continue;
            }
            double npc[3];
            for (int j = 0; j < 3; j++) {
                npc[j] = X0[j]*nps[0] + Y0[j]*nps[1] + NS[j]*nps[2];
            }
            double dot = npc[0]*np_v[0] + npc[1]*np_v[1] + npc[2]*np_v[2];
            dot = constrain(dot, -1.0, 1.0);
            double ang_deg = acos(dot) * (180.0 / PI);
            sum_sq += ang_deg * ang_deg;
            n_rms++;
        }
        double rms_deg = sqrt(sum_sq / n_rms);
        rotation_data->rms_deg = (float)rms_deg;

        //DISCARD CHECK 3: fit too poor (also rejects NaN)
        if (!(rms_deg <= RMS_LIMIT_DEG)) {
            rotation_data->fit_status = FIT_RMS_TOO_HIGH;
            return; //Keep old msx, msy, theta0
        }
    }

    if (!(fabs(msx) < 1e9 && fabs(msy) < 1e9 && fabs(theta0) <= 360.0)) { //Also rejects NaN/inf
        rotation_data->fit_status = FIT_DEGENERATE;
        return;
    }

    //Fit accepted
    rotation_data->msx = msx;
    rotation_data->msy = msy;
    rotation_data->theta0 = theta0;
    rotation_data->fit_status = FIT_OK;
}
