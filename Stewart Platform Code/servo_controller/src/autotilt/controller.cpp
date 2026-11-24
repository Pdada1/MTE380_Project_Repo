#include "controller.h"
#include "LinearFit.h" //SimpleRegression library
#include "ParabolicFit.h"

namespace autotilt {

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

void update_shifted_tilt(RotationData* rd, float t_new, float thetax, float thetay) {
    const int hist_n = FIT_HISTORY_SAMPLES;
    rd->shifted_valid = false; //Set again below if the lookback succeeds

    //Shift the tilt history and append the new sample
    for (int i = 0; i < hist_n - 1; i++) {
        rd->t[i]      = rd->t[i + 1];
        rd->thetax[i] = rd->thetax[i + 1];
        rd->thetay[i] = rd->thetay[i + 1];
    }
    rd->t[hist_n - 1]      = t_new;
    rd->thetax[hist_n - 1] = thetax;
    rd->thetay[hist_n - 1] = thetay;
    if (rd->num_valid < hist_n) {
        rd->num_valid++;
    }

    //Linearly interpolate the tilt history CONTROLLER_DELAY earlier
    float t_target = t_new - CONTROLLER_DELAY;
    int first_valid = hist_n - rd->num_valid;
    int j = hist_n - 2;
    while (j >= first_valid && rd->t[j] > t_target) {
        j--;
    }
    if (j < first_valid) {
        return;
    }

    float dt = rd->t[j + 1] - rd->t[j];
    float frac = (dt > 1e-9f) ? (t_target - rd->t[j]) / dt : 0.0f;
    frac = constrain(frac, 0.0f, 1.0f);

    rd->thetax_shifted = rd->thetax[j] + (rd->thetax[j + 1] - rd->thetax[j]) * frac;
    rd->thetay_shifted = rd->thetay[j] + (rd->thetay[j + 1] - rd->thetay[j]) * frac;
    rd->shifted_valid = true;
}


//============================================================
// Wahba fit of msx, msy with theta0 fixed at 0
// (port of YES_msx_msy_wahba.py)
//============================================================

static void cross3(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

//out = M * v, where M is stored by columns: M[k] is the k-th column
static void apply_cols(const float M[3][3], const float v[3], float out[3]) {
    for (int i = 0; i < 3; i++) {
        out[i] = v[0]*M[0][i] + v[1]*M[1][i] + v[2]*M[2][i];
    }
}

//R(msx, msy) and its partial derivatives, all stored by columns.
//Columns of R are x0 = s, y0 = ns x s, ns, with s = (1, 0, msx)/norm and ns = (-msx, -msy, 1)/norm
//(python: rotation_from_ms).
struct MsFrame {
    float R[3][3];
    float dRx[3][3]; //dR/dmsx
    float dRy[3][3]; //dR/dmsy
};

static void ms_frame(float msx, float msy, MsFrame* f) {
    float un = sqrtf(1.0f + msx*msx);
    float vn = sqrtf(msx*msx + msy*msy + 1.0f);
    float s[3]  = {1.0f/un, 0.0f, msx/un};
    float ns[3] = {-msx/vn, -msy/vn, 1.0f/vn};

    //d(u/|u|) = (I - s s^T) du / |u|, with du = (0,0,1) for msx
    float ds[3] = {-s[0]*s[2]/un, -s[1]*s[2]/un, (1.0f - s[2]*s[2])/un};
    //d(v/|v|) = (I - ns ns^T) dv / |v|, with dv = (-1,0,0) for msx and (0,-1,0) for msy
    float dnx[3] = {(-1.0f + ns[0]*ns[0])/vn, ns[1]*ns[0]/vn, ns[2]*ns[0]/vn};
    float dny[3] = {ns[0]*ns[1]/vn, (-1.0f + ns[1]*ns[1])/vn, ns[2]*ns[1]/vn};

    float y0[3], t1[3], t2[3], dy0x[3], dy0y[3];
    cross3(ns, s, y0);
    cross3(dnx, s, t1);
    cross3(ns, ds, t2);
    for (int i = 0; i < 3; i++) dy0x[i] = t1[i] + t2[i];
    cross3(dny, s, dy0y);

    for (int i = 0; i < 3; i++) {
        f->R[0][i] = s[i];    f->R[1][i] = y0[i];     f->R[2][i] = ns[i];
        f->dRx[0][i] = ds[i]; f->dRx[1][i] = dy0x[i]; f->dRx[2][i] = dnx[i];
        f->dRy[0][i] = 0.0f;  f->dRy[1][i] = dy0y[i]; f->dRy[2][i] = dny[i];
    }
}

//Builds the (np, nps) pair for one sample (python: build_vector_pairs). Returns false if the sample is unusable
//(az undefined, or ax = ay = 0 giving a zero cross product).
static bool wahba_make_sample(float ax, float ay, float nx, float ny, WahbaSample* out) {
    //nps = (-sin(thetax), -sin(thetay), 1) / norm
    float nn = sqrtf(nx*nx + ny*ny + 1.0f);
    out->nps[0] = nx/nn;
    out->nps[1] = ny/nn;
    out->nps[2] = 1.0f/nn;

    //az = (c - sqrt(c^2 - 4(ax^2 + ay^2))) / 2
    float disc = WAHBA_AZ_CONST*WAHBA_AZ_CONST - 4.0f*(ax*ax + ay*ay);
    if (!(disc >= 0.0f)) { //Also rejects NaN
        return false;
    }
    float az = (WAHBA_AZ_CONST - sqrtf(disc)) / 2.0f;

    //np = (a x (-ay, ax, 0)) / |...|, with a = (ax, ay, -az)
    float a[3] = {ax, ay, -az};
    float perp[3] = {-ay, ax, 0.0f};
    float n[3];
    cross3(a, perp, n);
    float len = sqrtf(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
    if (!(len >= 1e-12f)) {
        return false;
    }
    out->np[0] = n[0]/len;
    out->np[1] = n[1]/len;
    out->np[2] = n[2]/len;

    //Measured horizontal direction
    float an = sqrtf(ax*ax + ay*ay);
    if (an < 1e-12f) an = 1e-12f;
    out->ahat[0] = ax/an;
    out->ahat[1] = ay/an;
    return true;
}

//Combined R^2 between measured and predicted ahatx/ahaty over the stored samples (python: calculate_ahat + calculate_r_squared)
static float wahba_r2(const RotationData* rd, float msx, float msy) {
    MsFrame f;
    ms_frame(msx, msy, &f);
    int n = rd->wahba_count;

    float mean = 0;
    for (int i = 0; i < n; i++) mean += rd->wahba[i].ahat[0] + rd->wahba[i].ahat[1];
    mean /= (2.0f*n);

    float ss_res = 0, ss_tot = 0;
    for (int i = 0; i < n; i++) {
        float rv[3];
        apply_cols(f.R, rd->wahba[i].nps, rv);
        float norm = sqrtf(rv[0]*rv[0] + rv[1]*rv[1]);
        if (norm < 1e-12f) norm = 1e-12f;
        for (int k = 0; k < 2; k++) {
            float m = rd->wahba[i].ahat[k];
            float d = m - rv[k]/norm;
            ss_res += d*d;
            ss_tot += (m - mean)*(m - mean);
        }
    }
    if (ss_tot < 1e-15f) return NAN;
    return 1.0f - ss_res/ss_tot;
}

//Running sums B = sum np nps^T and C = sum nps nps^T over the stored samples. For unit vectors
//  sum |np_i - R nps_i|^2 = 2N - 2 sum_jk R_jk B_jk,   gradient_a = -sum_jk dR_a,jk B_jk,
//  (J^T J)_ab = sum_kl C_kl (dR_a[col k] . dR_b[col l])
//so a fit iteration costs the same no matter how many samples are stored.
static void wahba_add_sums(RotationData* rd, const WahbaSample* s, float sign) {
    for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
            rd->wahba_B[j][k] += sign * s->np[j]  * s->nps[k];
            rd->wahba_C[j][k] += sign * s->nps[j] * s->nps[k];
        }
    }
}

static void wahba_rebuild_sums(RotationData* rd) {
    for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
            rd->wahba_B[j][k] = 0;
            rd->wahba_C[j][k] = 0;
        }
    }
    for (int i = 0; i < rd->wahba_count; i++) {
        wahba_add_sums(rd, &rd->wahba[i], 1.0f);
    }
    rd->wahba_since_rebuild = 0;
}

static float dot3(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

//Two-variable nonlinear least squares for (msx, msy) minimizing sum |np_i - R(msx,msy) nps_i|^2 (python: fit_ms_wahba),
//solved by Gauss-Newton on the running sums. Warm-started from the previous fit; the minimum is the same one
//the python finds from (0, 0).
static void wahba_fit(RotationData* rd) {
    float p[2] = {0.0f, 0.0f};
    if (rd->fit_status == WAHBA_OK || rd->fit_status == WAHBA_NOT_CONVERGED) {
        p[0] = rd->msx;
        p[1] = rd->msy;
    }
    bool converged = false;
    int iter;

    for (iter = 0; iter < WAHBA_MAX_ITER && !converged; iter++) {
        MsFrame f;
        ms_frame(p[0], p[1], &f);

        //b_a = sum_jk dR_a,jk B_jk  (dR stored by columns: dR[k][j] is row j, column k)
        float b1 = 0, b2 = 0;
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                b1 += f.dRx[k][j] * rd->wahba_B[j][k];
                b2 += f.dRy[k][j] * rd->wahba_B[j][k];
            }
        }

        //A_ab = sum_kl C_kl (dR_a[col k] . dR_b[col l])
        float a11 = 0, a12 = 0, a22 = 0;
        for (int k = 0; k < 3; k++) {
            for (int l = 0; l < 3; l++) {
                float C = rd->wahba_C[k][l];
                a11 += C * dot3(f.dRx[k], f.dRx[l]);
                a12 += C * dot3(f.dRx[k], f.dRy[l]);
                a22 += C * dot3(f.dRy[k], f.dRy[l]);
            }
        }

        //Gauss-Newton step: A * step = -g with g = -b, i.e. A * step = b (tiny damping keeps A invertible)
        float m11 = a11 * (1.0f + 1e-6f) + 1e-12f;
        float m22 = a22 * (1.0f + 1e-6f) + 1e-12f;
        float det = m11*m22 - a12*a12;
        if (!(fabsf(det) > 1e-20f)) { //Also catches NaN
            rd->fit_status = WAHBA_FAILED;
            return;
        }
        float s0 = ( m22*b1 - a12*b2) / det;
        float s1 = (-a12*b1 + m11*b2) / det;

        //Limit the step size
        float big = fmaxf(fabsf(s0), fabsf(s1));
        if (big > 0.1f) {
            s0 *= 0.1f/big;
            s1 *= 0.1f/big;
        }

        p[0] += s0;
        p[1] += s1;
        if (fabsf(s0) < WAHBA_STEP_TOL && fabsf(s1) < WAHBA_STEP_TOL) {
            converged = true;
        }
    }

    if (!(p[0] == p[0]) || !(p[1] == p[1])) { //NaN
        rd->fit_status = WAHBA_FAILED;
        return;
    }

    rd->msx = p[0];
    rd->msy = p[1];
    rd->theta0 = 0; //Fixed
    rd->fit_iters = iter;

    //Residual cost at the solution
    MsFrame f;
    ms_frame(p[0], p[1], &f);
    float cost;
#if WAHBA_COMPUTE_STATS
    cost = 0;
    for (int i = 0; i < rd->wahba_count; i++) {
        float rv[3];
        apply_cols(f.R, rd->wahba[i].nps, rv);
        for (int k = 0; k < 3; k++) {
            float d = rd->wahba[i].np[k] - rv[k];
            cost += d*d;
        }
    }
    rd->r2 = wahba_r2(rd, p[0], p[1]);
#else
    //2N - 2<R,B>: cheap, but float cancellation limits it to roughly 1e-3 in rms
    float rb = 0;
    for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
            rb += f.R[k][j] * rd->wahba_B[j][k];
        }
    }
    cost = fmaxf(2.0f*rd->wahba_count - 2.0f*rb, 0.0f);
    rd->r2 = NAN;
#endif
    rd->rms = sqrtf(cost / (3.0f*rd->wahba_count));
    rd->fit_status = converged ? WAHBA_OK : WAHBA_NOT_CONVERGED;
}

bool wahba_update(RotationData* rd, BallData* ball_data) {
    //Need a tilt that was really looked up CONTROLLER_DELAY back, not zero-filled history
    if (!rd->shifted_valid) {
        return false;
    }

    //Sample spacing
    rd->wahba_spacing_ctr++;
    if (rd->wahba_spacing_ctr < WAHBA_SAMPLE_SPACING) {
        return false;
    }
    rd->wahba_spacing_ctr = 0;

    //Acceleration and delay-shifted -sin(tilt), the columns of the python input
    float ax = acc_av(ball_data, 'x') * ACC_SCALE;
    float ay = acc_av(ball_data, 'y') * ACC_SCALE;
    float nx = -sinf(rd->thetax_shifted * (float)DEG2RAD);
    float ny = -sinf(rd->thetay_shifted * (float)DEG2RAD);

    WahbaSample s;
    if (!wahba_make_sample(ax, ay, nx, ny, &s)) {
        return false; //Skip unusable sample
    }

    //Store in the ring buffer (overwrites the oldest once full), keeping the running sums up to date
    if (rd->wahba_count == WAHBA_FIT_SAMPLES) {
        wahba_add_sums(rd, &rd->wahba[rd->wahba_head], -1.0f);
    }
    wahba_add_sums(rd, &s, 1.0f);
    rd->wahba[rd->wahba_head] = s;
    rd->wahba_head = (rd->wahba_head + 1) % WAHBA_FIT_SAMPLES;
    if (rd->wahba_count < WAHBA_FIT_SAMPLES) {
        rd->wahba_count++;
    }
    rd->wahba_since_fit++;
    if (++rd->wahba_since_rebuild >= WAHBA_REBUILD_EVERY) {
        wahba_rebuild_sums(rd);
    }

    if (rd->wahba_count < WAHBA_FIT_SAMPLES || rd->wahba_since_fit < WAHBA_FIT_INTERVAL) {
        return false;
    }
    rd->wahba_since_fit = 0;
    wahba_fit(rd);
    return true;
}

bool wahba_compensate_tilt(const RotationData* rd, float* thetax, float* thetay) {
#if WAHBA_USE_IN_CONTROL
    if (rd->fit_status != WAHBA_OK) {
        return false;
    }
    if (!(fabsf(rd->msx) <= WAHBA_MS_MAX) || !(fabsf(rd->msy) <= WAHBA_MS_MAX)) { //Also rejects NaN
        return false;
    }

    //Desired plane normal (unnormalized; only the ratios matter below), in the same form as nps: (-sin(thx), -sin(thy), 1)
    float n_des[3] = {
        -sinf(*thetax * (float)DEG2RAD),
        -sinf(*thetay * (float)DEG2RAD),
        1.0f
    };

    //w = R^T n_des (dot with each column of R)
    MsFrame f;
    ms_frame(rd->msx, rd->msy, &f);
    float w[3];
    for (int k = 0; k < 3; k++) {
        w[k] = f.R[k][0]*n_des[0] + f.R[k][1]*n_des[1] + f.R[k][2]*n_des[2];
    }
    if (w[2] <= 1e-6f) {
        return false;
    }

    //Invert nps = (-sin(thx), -sin(thy), 1)/norm  =>  -sin(thx) = w0/w2, -sin(thy) = w1/w2
    float sx = constrain(w[0]/w[2], -1.0f, 1.0f);
    float sy = constrain(w[1]/w[2], -1.0f, 1.0f);
    *thetax = -asinf(sx) / (float)DEG2RAD;
    *thetay = -asinf(sy) / (float)DEG2RAD;
    return true;
#else
    return false;
#endif
}

} // namespace autotilt
