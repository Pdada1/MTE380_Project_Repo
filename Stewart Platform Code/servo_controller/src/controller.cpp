#include "controller.h"
#include "LinearFit.h" //SimpleRegression library
#include "ParabolicFit.h"
#include "BasicLinearAlgebra.h" //BasicLinearAlgebra library
#include "lmmin.h" //lmfit library
using namespace BLA;

#define POS_SAMPLES (int)(1) //1 sample
#define VEL_SAMPLES (int)(4) //4 samples
#define ACC_SAMPLES (int)(12) //12 samples
#define DEG2RAD 0.017453292519943295f

struct ResidualMeasurements {
    double ax_measured[FIT_SAMPLES] = {0};
    double ay_measured[FIT_SAMPLES] = {0};
    double thetax[FIT_SAMPLES] = {0};
    double thetay[FIT_SAMPLES] = {0};
    // new, precomputed once per fit:
    float ahx[FIT_SAMPLES], ahy[FIT_SAMPLES];
    float p0[FIT_SAMPLES], p1[FIT_SAMPLES], p2[FIT_SAMPLES];
};

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
void read_rotation_data(RotationData* rotation_data, BallData* ball_data) {
    //Shift data arrays to make room for new sample
    for (int i=0; i<FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1; i++) {
        rotation_data->ax[i] = rotation_data->ax[i+1];
        rotation_data->ay[i] = rotation_data->ay[i+1];
        rotation_data->thetax[i] = rotation_data->thetax[i+1];
        rotation_data->thetay[i] = rotation_data->thetay[i+1];
        rotation_data->t[i] = rotation_data->t[i+1];
    }

    //Add new sample to end of arrays
    rotation_data->ax[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = acc_av(ball_data, 'x');
    rotation_data->ay[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = acc_av(ball_data, 'y');
    rotation_data->thetax[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = -sin(pid_get_xtilt(ball_data)*PI/180);
    rotation_data->thetay[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = -sin(pid_get_ytilt(ball_data)*PI/180);
    rotation_data->t[FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1] = ball_data->t[CAMERA_SAMPLES-1];
}

void precompute_measurements(ResidualMeasurements* m) {
    for (int i = 0; i < FIT_SAMPLES; i++) {
        float ax = m->ax_measured[i], ay = m->ay_measured[i];
        float n = sqrtf(ax*ax + ay*ay);
        if (n < 1e-8f) { m->ahx[i] = 0; m->ahy[i] = 0; }
        else           { m->ahx[i] = ax / n; m->ahy[i] = ay / n; }

        float sx = -sinf(m->thetax[i] * DEG2RAD);
        float sy = -sinf(m->thetay[i] * DEG2RAD);
        float inv = 1.0f / sqrtf(sx*sx + sy*sy + 1.0f);
        m->p0[i] = sx * inv;
        m->p1[i] = sy * inv;
        m->p2[i] = inv;
    }
}

// float norm(const Matrix<3>& v) {
//     return sqrt(v(0)*v(0) + v(1)*v(1) + v(2)*v(2));
// }

// Matrix<3> cross(const Matrix<3>& a, const Matrix<3>& b) {
//     Matrix<3> c;
//     c(0) = a(1) * b(2) - a(2) * b(1);
//     c(1) = a(2) * b(0) - a(0) * b(2);
//     c(2) = a(0) * b(1) - a(1) * b(0);
//     return c;
// }

// void find_one_residual(const double* guesses, double* measurements, double* residuals) { //Residual given msx, msy, theta0
//     float msx = guesses[0];
//     float msy = guesses[1];
//     float theta0_rad = guesses[2]*PI/180;
//     float ax_measured = measurements[0];
//     float ay_measured = measurements[1];
//     float thetax = measurements[2];
//     float thetay = measurements[3];

//     float ahatx_measured;
//     float ahaty_measured;
//     if (abs(ax_measured) < 1e-8 && abs(ay_measured) < 1e-8) { //0
//         ahatx_measured = 0;
//         ahaty_measured = 0;
//     }
//     else {
//         ahatx_measured = ax_measured / sqrt(ax_measured*ax_measured + ay_measured*ay_measured);
//         ahaty_measured = ay_measured / sqrt(ax_measured*ax_measured + ay_measured*ay_measured);
//     }

//     Matrix<3> nps = {-sin(thetax*PI/180), -sin(thetay*PI/180), 1};
//     Matrix<3> s = {1, 0, msx};
//     Matrix<3> ns = {msx, msy, -1};
//     nps = nps/norm(nps);
//     s = s/norm(s);
//     ns = ns/norm(ns);

//     Matrix<3> x0 = s*(float)cos(theta0_rad) + cross(ns,s)*(float)sin(theta0_rad);
//     Matrix<3> y0 = cross(ns,s)*(float)cos(theta0_rad) - s*(float)sin(theta0_rad);

//     Matrix<3,3> A(1, 0, 0,
//                 0, 1, 0,
//                 0, 0, 0);
//     Matrix<3,3> R(x0(0), y0(0), ns(0),
//                 x0(1), y0(1), ns(1),
//                 x0(2), y0(2), ns(2));

//     Matrix<3> ahat_calculated = A*R*nps;
//     ahat_calculated = ahat_calculated/norm(ahat_calculated);
//     float ahatx_calculated = ahat_calculated(0);
//     float ahaty_calculated = ahat_calculated(1);

//     residuals[0] = ahatx_measured - ahatx_calculated;
//     residuals[1] = ahaty_measured - ahaty_calculated;
// }

// void find_all_residuals(const double *p, int n, const void *data, double *f, int *info) {
//     const double* guesses = p;
//     ResidualMeasurements* all_residual_measurements = (ResidualMeasurements*)data;
//     double* all_residuals = f;

//     double residuals[2] = {0};
//     double measurements[4] = {0};
//     for (int i=0; i<FIT_SAMPLES; i++) {
//         measurements[0] = all_residual_measurements->ax_measured[i];
//         measurements[1] = all_residual_measurements->ay_measured[i];
//         measurements[2] = all_residual_measurements->thetax[i];
//         measurements[3] = all_residual_measurements->thetay[i];
//         find_one_residual(guesses, measurements, residuals);
//         all_residuals[2*i] = residuals[0];
//         all_residuals[2*i+1] = residuals[1];
//     }
// }

// Computes cost = sum f^2, and (if wantJ) JtJ and Jtf, in a single pass.
static float eval_fit(const float q[3], const ResidualMeasurements* m,
                      bool wantJ, float JtJ[3][3], float Jtf[3]) {
    const float a = q[0], b = q[1];
    const float c = cosf(q[2]), sn = sinf(q[2]);

    const float invs  = 1.0f / sqrtf(1.0f + a*a);
    const float invs3 = invs*invs*invs;
    const float invn  = 1.0f / sqrtf(a*a + b*b + 1.0f);
    const float invn3 = invn*invn*invn;

    const float s0 = invs, s2 = a*invs;
    const float n0 = a*invn, n1 = b*invn, n2 = -invn;
    const float w0 = n1*s2;
    const float w1 = n2*s0 - n0*s2;

    // parameter derivatives (only needed for the Jacobian)
    float ds0_a=0, dw0_a=0, dw0_b=0, dw1_a=0, dw1_b=0;
    float dn0_a=0, dn1_a=0, dn0_b=0, dn1_b=0;
    if (wantJ) {
        ds0_a = -a*invs3;
        const float ds2_a = invs3;
        dn0_a = invn - a*a*invn3;   dn1_a = -a*b*invn3;
        const float dn2_a = a*invn3;
        dn0_b = -a*b*invn3;         dn1_b = invn - b*b*invn3;
        const float dn2_b = b*invn3;
        dw0_a = dn1_a*s2 + n1*ds2_a;
        dw0_b = dn1_b*s2;
        dw1_a = dn2_a*s0 + n2*ds0_a - dn0_a*s2 - n0*ds2_a;
        dw1_b = dn2_b*s0 - dn0_b*s2;
        for (int i=0;i<3;i++){ Jtf[i]=0; for(int j=0;j<3;j++) JtJ[i][j]=0; }
    }

    float cost = 0;
    for (int i = 0; i < FIT_SAMPLES; i++) {
        const float p0 = m->p0[i], p1 = m->p1[i], p2 = m->p2[i];
        const float ahx = m->ahx[i], ahy = m->ahy[i];
        const float u = c*p0 - sn*p1;
        const float v = sn*p0 + c*p1;

        const float rx = s0*u + w0*v + n0*p2;
        const float ry =         w1*v + n1*p2;
        const float f  = ahx*ry - ahy*rx;
        cost += f*f;

        if (wantJ) {
            const float drx_a = ds0_a*u + dw0_a*v + dn0_a*p2;
            const float dry_a =           dw1_a*v + dn1_a*p2;
            const float drx_b =           dw0_b*v + dn0_b*p2;
            const float dry_b =           dw1_b*v + dn1_b*p2;
            const float drx_t = w0*u - s0*v;
            const float dry_t = w1*u;

            const float J0 = ahx*dry_a - ahy*drx_a;
            const float J1 = ahx*dry_b - ahy*drx_b;
            const float J2 = ahx*dry_t - ahy*drx_t;

            JtJ[0][0]+=J0*J0; JtJ[0][1]+=J0*J1; JtJ[0][2]+=J0*J2;
            JtJ[1][1]+=J1*J1; JtJ[1][2]+=J1*J2; JtJ[2][2]+=J2*J2;
            Jtf[0]+=J0*f; Jtf[1]+=J1*f; Jtf[2]+=J2*f;
        }
    }
    if (wantJ) { JtJ[1][0]=JtJ[0][1]; JtJ[2][0]=JtJ[0][2]; JtJ[2][1]=JtJ[1][2]; }
    return cost;
}

// Solve A x = rhs (3x3) by Cramer's rule. Returns false if singular.
static bool solve3(float A[3][3], float rhs[3], float x[3]) {
    float tr = A[0][0] + A[1][1] + A[2][2];
    if (tr < 1e-30f) return false;
    float k = 1.0f / tr;                       // normalize to avoid tiny/huge numbers
    float a00=A[0][0]*k, a01=A[0][1]*k, a02=A[0][2]*k;
    float a11=A[1][1]*k, a12=A[1][2]*k, a22=A[2][2]*k;
    float b0=rhs[0]*k, b1=rhs[1]*k, b2=rhs[2]*k;

    float c00 = a11*a22 - a12*a12;
    float c01 = a02*a12 - a01*a22;
    float c02 = a01*a12 - a02*a11;
    float det = a00*c00 + a01*c01 + a02*c02;
    if (fabsf(det) < 1e-12f) return false;
    float id = 1.0f / det;
    float c11 = a00*a22 - a02*a02;
    float c12 = a01*a02 - a00*a12;
    float c22 = a00*a11 - a01*a01;
    x[0] = (c00*b0 + c01*b1 + c02*b2) * id;
    x[1] = (c01*b0 + c11*b1 + c12*b2) * id;
    x[2] = (c02*b0 + c12*b1 + c22*b2) * id;
    return true;
}

// Levenberg-Marquardt. q = {msx, msy, theta_rad}. Returns iterations used, or -1 on failure.
static int fit_lm(const ResidualMeasurements* m, float q[3], int max_iter) {
    float JtJ[3][3], Jtf[3];
    float cost = eval_fit(q, m, true, JtJ, Jtf);
    if (cost < 1e-9f) return 0;
    float lambda = 1e-3f;
    int rejects = 0;

    for (int it = 0; it < max_iter; it++) {
        float tr = JtJ[0][0] + JtJ[1][1] + JtJ[2][2];
        float A[3][3], rhs[3], d[3];
        for (int i=0;i<3;i++) {
            for (int j=0;j<3;j++) A[i][j] = JtJ[i][j];
            A[i][i] += lambda*JtJ[i][i] + 1e-4f*tr;   // stronger ridge (was 1e-6)
            rhs[i] = -Jtf[i];
        }
        if (!solve3(A, rhs, d)) return -1;

        // converged: step is negligible
        if (fabsf(d[0]) < 2e-5f && fabsf(d[1]) < 2e-5f && fabsf(d[2]) < 2e-4f)
            return it + 1;

        float qn[3] = { q[0]+d[0], q[1]+d[1], q[2]+d[2] };
        float cost_n = eval_fit(qn, m, false, JtJ, Jtf);   // cost only, JtJ/Jtf untouched

        if (cost_n < cost) {
            bool done = (cost - cost_n) < 1e-3f * cost;
            q[0]=qn[0]; q[1]=qn[1]; q[2]=qn[2];
            cost = cost_n;
            if (done) return it + 1;
            eval_fit(q, m, true, JtJ, Jtf);                // Jacobian only at accepted points
            lambda *= 0.3f; if (lambda < 1e-7f) lambda = 1e-7f;
            rejects = 0;
        } else {
            lambda *= 10.0f;
            if (++rejects >= 3) return it + 1;
        }
    }
    return max_iter;
}

void fit_rotation_data(RotationData* rotation_data) {
    //Check if there are enough samples after CONTROLLER_DELAY to make up FIT_SAMPLES
    int t_start_index = 0; // Index after CONTROLLER_DELAY
    int t_end_index = FIT_SAMPLES+(int)(CONTROLLER_DELAY_MAX/SAMPLE_RATE_APPROX)-1;
    for (int i=0; i<=t_end_index; i++) {
        t_start_index = i;
        if (rotation_data->t[i] - rotation_data->t[0] >= CONTROLLER_DELAY) {
            break;
        }
    }
    if (t_end_index - t_start_index < FIT_SAMPLES - 1) {
        Serial.println(F("Not enough samples to fit rotation data!"));
        return;
    }
    else {
        t_start_index = t_end_index - FIT_SAMPLES + 1;
    }

    //Window of FIT_SAMPLES
    float t[FIT_SAMPLES] = {0};
    float ax[FIT_SAMPLES] = {0};
    float ay[FIT_SAMPLES] = {0};
    for (int i=0; i<FIT_SAMPLES; i++) {
        t[i] = rotation_data->t[t_start_index+i] - rotation_data->t[t_start_index]; //Use difference to keep numbers from getting large
        ax[i] = rotation_data->ax[t_start_index+i];
        ay[i] = rotation_data->ay[t_start_index+i];
    }

    //Shift thetax and thetay by CONTROLLER_DELAY, then interpolate to match the t[] array
    float thetax[FIT_SAMPLES] = {0};
    float thetay[FIT_SAMPLES] = {0};
    float t_shifted[FIT_SAMPLES] = {0};
    for (int i=0; i<FIT_SAMPLES; i++) {
        t_shifted[i] = t[i] + CONTROLLER_DELAY;
    }
    int sample1 = 0;
    int sample2 = 1;
    for (int i=0; i<FIT_SAMPLES; i++) {
        //Make sample1 and sample2 bracket t[i]
        while (sample2 < FIT_SAMPLES-1 && t_shifted[sample2] < t[i]) {
            sample1++;
            sample2++;
        }

        //Interpolate between sample1 and sample2 to t[i]
        float dt = t_shifted[sample2] - t_shifted[sample1];
        float frac = (dt > 1e-9f) ? (t[i] - t_shifted[sample1]) / dt : 0.0f;

        thetax[i] = rotation_data->thetax[t_start_index+sample1] + (rotation_data->thetax[t_start_index+sample2] - rotation_data->thetax[t_start_index+sample1]) * frac;
        thetay[i] = rotation_data->thetay[t_start_index+sample1] + (rotation_data->thetay[t_start_index+sample2] - rotation_data->thetay[t_start_index+sample1]) * frac;
    }

    //Debugging
    // for (int i=0; i<FIT_SAMPLES; i++) {
    //     Serial.print(rotation_data->thetax[i], 3);
    //     Serial.print(' ');
    // }
    // Serial.println();
    // for (int i=0; i<FIT_SAMPLES; i++) {
    //     Serial.print(thetax[i], 3);
    //     Serial.print(' ');
    // }
    // Serial.println();
    rotation_data->thetax_shifted = thetax[FIT_SAMPLES-1];
    rotation_data->thetay_shifted = thetay[FIT_SAMPLES-1];

    // Now we have ax,ay,thetax,thetay compensated for CONTROLLER_DELAY, all at the same time instances.
    // // Least squares:
    // ResidualMeasurements measurements;
    // for (int i = 0; i < FIT_SAMPLES; i++) {
    //     measurements.ax_measured[i] = ax[i];
    //     measurements.ay_measured[i] = ay[i];
    //     measurements.thetax[i] = thetax[i];
    //     measurements.thetay[i] = thetay[i];
    // }
    // precompute_measurements(&measurements);

    // // skip fits with no signal
    // float energy = 0;
    // for (int i = 0; i < FIT_SAMPLES; i++) energy += ax[i]*ax[i] + ay[i]*ay[i];
    // if (energy < 0.01f) return;      // tune: print energy at idle vs. motion

    // // warm start from last good fit (stored in radians)
    // static float q_prev[3] = {0, 0, 0};
    // float q[3] = { q_prev[0], q_prev[1], q_prev[2] };

    // unsigned long t0 = micros();
    // int iters = fit_lm(&measurements, q, 8);
    // Serial.print(F("fit us: "));   Serial.println(micros() - t0);
    // Serial.print(F("iters: "));    Serial.println(iters);

    // // accept only sane results
    // if (iters > 0 && fabsf(q[0]) < 0.2f && fabsf(q[1]) < 0.2f && fabsf(q[2]) < 0.5f) {
    //     q_prev[0] = q[0]; q_prev[1] = q[1]; q_prev[2] = q[2];
    //     rotation_data->msx = q[0];
    //     rotation_data->msy = q[1];
    //     rotation_data->theta0 = q[2] / DEG2RAD;     // back to degrees
    // }
}

void find_ahat(float msx, float msy, float theta0, float thetax, float thetay, float* ahat) { //For logging/debugging
    // float theta0_rad = theta0*PI/180;

    // Matrix<3> nps = {-sin(thetax*PI/180), -sin(thetay*PI/180), 1};
    // Matrix<3> s = {1, 0, msx};
    // Matrix<3> ns = {msx, msy, -1};
    // nps = nps/norm(nps);
    // s = s/norm(s);
    // ns = ns/norm(ns);

    // Matrix<3> x0 = s*(float)cos(theta0_rad) + cross(ns,s)*(float)sin(theta0_rad);
    // Matrix<3> y0 = cross(ns,s)*(float)cos(theta0_rad) - s*(float)sin(theta0_rad);

    // Matrix<3,3> A(1, 0, 0,
    //             0, 1, 0,
    //             0, 0, 0);
    // Matrix<3,3> R(x0(0), y0(0), ns(0),
    //             x0(1), y0(1), ns(1),
    //             x0(2), y0(2), ns(2));

    // Matrix<3> ahat_calculated = A*R*nps;
    // ahat_calculated = ahat_calculated/norm(ahat_calculated);
    // float ahatx_calculated = ahat_calculated(0);
    // float ahaty_calculated = ahat_calculated(1);

    // ahat[0] = ahatx_calculated;
    // ahat[1] = ahaty_calculated;
}