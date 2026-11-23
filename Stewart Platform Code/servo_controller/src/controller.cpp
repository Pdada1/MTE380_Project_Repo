#include "controller.h"
#include "LinearFit.h" //SimpleRegression library
#include "ParabolicFit.h"

#define POS_SAMPLES (int)(1) //1 sample
#define VEL_SAMPLES (int)(4) //4 samples
#define ACC_SAMPLES (int)(12) //12 samples

float int_ex = 0;
float int_ey = 0;

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

float inte(BallData* ball_data, char axis) { //Make sure to only call once (for 'x' and then 'y') per new sample, otherwise integral will be inaccurate
    float e;
    float dt = ball_data->t[CAMERA_SAMPLES-1] - ball_data->t[CAMERA_SAMPLES-2];
    switch (axis) {
        case 'x':
            e = ball_data->x[CAMERA_SAMPLES-1] - ball_data->xr[CAMERA_SAMPLES-1];
            int_ex += e*dt;
            int_ex = constrain(int_ex, -I_LIMIT, I_LIMIT);
            return int_ex;
        case 'y':
            e = ball_data->y[CAMERA_SAMPLES-1] - ball_data->yr[CAMERA_SAMPLES-1];
            int_ey += e*dt;
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

float pid_get_tilt(BallData* ball_data, char axis) { //Gets tilt for axis 'x' or 'y'
    float e = e_av(ball_data, axis);
    float int_e = inte(ball_data, axis);
    float d_e = vel_av(ball_data, axis);

    float u = e*KP + int_e*KI + d_e*KD;
    return u;
}

float pid_get_xtilt(BallData* ball_data) {
    return pid_get_tilt(ball_data, 'x');
}

float pid_get_ytilt(BallData* ball_data) {
    return pid_get_tilt(ball_data, 'y');
}

//ROTATION FITTING
//ROTATION FITTING
void read_rotation_data(RotationData* rotation_data, BallData* ball_data, float thetax, float thetay) {
    const int N = ROT_BUF_SIZE;

    //Shift data arrays to make room for new sample
    for (int i=0; i<N-1; i++) {
        rotation_data->ax[i] = rotation_data->ax[i+1];
        rotation_data->ay[i] = rotation_data->ay[i+1];
        rotation_data->thetax[i] = rotation_data->thetax[i+1];
        rotation_data->thetay[i] = rotation_data->thetay[i+1];
        rotation_data->t[i] = rotation_data->t[i+1];
    }

    //Add new sample to end of arrays
    rotation_data->ax[N-1] = acc_av(ball_data, 'x') * (S/1000); //Scale back to m/s^2
    rotation_data->ay[N-1] = acc_av(ball_data, 'y') * (S/1000);
    rotation_data->thetax[N-1] = thetax;
    rotation_data->thetay[N-1] = thetay;
    rotation_data->t[N-1] = ball_data->t[CAMERA_SAMPLES-1];
    if (rotation_data->n_valid < N) rotation_data->n_valid++;
}

void fit_rotation_data(RotationData* rotation_data) {
    const int N = ROT_BUF_SIZE;
    const int w0 = N - FIT_SAMPLES;                //First index of the fit window (newest FIT_SAMPLES samples)
    const int oldest = N - rotation_data->n_valid; //First index holding real data

    //Need a full window, plus history reaching back CONTROLLER_DELAY before the window start
    if (rotation_data->n_valid < FIT_SAMPLES ||
        rotation_data->t[w0] - CONTROLLER_DELAY < rotation_data->t[oldest]) {
        Serial.println(F("Not enough samples to fit rotation data!"));
        return;
    }

    //Window of FIT_SAMPLES for acceleration
    float ax[FIT_SAMPLES] = {0};
    float ay[FIT_SAMPLES] = {0};
    for (int i=0; i<FIT_SAMPLES; i++) {
        ax[i] = rotation_data->ax[w0+i];
        ay[i] = rotation_data->ay[w0+i];
    }

    //For each window sample, look up the tilt commanded CONTROLLER_DELAY earlier,
    //interpolating within the stored history (which extends before the window)
    float thetax[FIT_SAMPLES] = {0};
    float thetay[FIT_SAMPLES] = {0};
    int j = oldest;
    for (int i=0; i<FIT_SAMPLES; i++) {
        float target = rotation_data->t[w0+i] - CONTROLLER_DELAY;

        //Make j and j+1 bracket target (targets increase monotonically)
        while (j < N-2 && rotation_data->t[j+1] < target) {
            j++;
        }

        float dt = rotation_data->t[j+1] - rotation_data->t[j];
        float frac = (dt > 1e-9f) ? (target - rotation_data->t[j]) / dt : 0.0f;
        frac = constrain(frac, 0.0f, 1.0f);

        thetax[i] = rotation_data->thetax[j] + (rotation_data->thetax[j+1] - rotation_data->thetax[j]) * frac;
        thetay[i] = rotation_data->thetay[j] + (rotation_data->thetay[j+1] - rotation_data->thetay[j]) * frac;
    }

    //Debugging
    rotation_data->thetax_shifted = thetax[FIT_SAMPLES-1];
    rotation_data->thetay_shifted = thetay[FIT_SAMPLES-1];

    // Now we have ax[],ay[],thetax[],thetay[] compensated for CONTROLLER_DELAY, all at the same time instances.
    double msx, msy, theta0;
    {
        const double AZ_CONST = 5.89;

        /* ---------- 1. attitude profile matrix B = sum( np * nps^T ) ---------- */
        double B[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
        int n_used = 0;

        for (int i = 0; i < FIT_SAMPLES; i++) {

            /* nps = (-sin(thetax), -sin(thetay), 1) / norm */
            double nps[3];
            nps[0] = -sin(thetax[i]*PI/180);
            nps[1] = -sin(thetay[i]*PI/180);
            nps[2] = 1.0;
            double nps_len = sqrt(nps[0]*nps[0] + nps[1]*nps[1] + 1.0);
            nps[0] /= nps_len;  nps[1] /= nps_len;  nps[2] /= nps_len;

            /* az = (c - sqrt(c^2 - 4(ax^2+ay^2))) / 2   (minus root) */
            double r2   = ax[i]*ax[i] + ay[i]*ay[i];
            double disc = AZ_CONST*AZ_CONST - 4.0*r2;
            if (disc < 0.0 || r2 < 1e-24) {
                continue;                       /* az or np undefined: skip sample */
            }
            double az = (AZ_CONST - sqrt(disc)) / 2.0;

            /* np = a x (-ay, ax, 0) with a = (ax, ay, -az), normalized.
            The cross product works out to (az*ax, az*ay, ax^2+ay^2). */
            double np_v[3];
            np_v[0] = az * ax[i];
            np_v[1] = az * ay[i];
            np_v[2] = r2;
            double np_len = sqrt(np_v[0]*np_v[0] + np_v[1]*np_v[1] + np_v[2]*np_v[2]);
            np_v[0] /= np_len;  np_v[1] /= np_len;  np_v[2] /= np_len;

            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    B[j][k] += np_v[j] * nps[k];
                }
            }
            n_used++;
        }

        if (n_used < 2) {
            msx = NAN;  msy = NAN;  theta0 = NAN;      /* not enough usable samples */
        } else {

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
                if (off < 1e-30) {
                    break;
                }
                for (int p = 0; p < 3; p++) {
                    for (int q = p + 1; q < 4; q++) {
                        if (fabs(K[p][q]) < 1e-300) {
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
            theta0 = atan2(x0_dot_c, x0_dot_s) * (180.0 / 3.14159265358979323846);
        }
    }
    rotation_data->msx = msx;
    rotation_data->msy = msy;
    rotation_data->theta0 = theta0;
}
