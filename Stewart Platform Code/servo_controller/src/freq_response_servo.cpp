/*Overall functionality needed
Wait for PC to prompt for calibration through serial message. 
Start by moving the plaform at x hertz 
Get the camera to record position data for the ball across multiple oscillation cycles of the ball
Prompt from computer to stop data recoding or just use a set amount of time 
Reset the platform to level and wait for the computer to tell it to continue?
*/
#include <camera_servo.h>
#define MIN_TSTEP 0.05f
#define MAX_ANGLE_DEVIATION 10.0f
#define MAX_FREQ 1.5f //Hz
#define MIN_FREQ 0.5f //Hz
#define FREQ_INCREMENT 0.1f //Hz 
#define NUM_OSCILLATIONS 5

#define SERVO_0_CH 8
#define SERVO_1_CH 10
#define SERVO_2_CH 9

#define BALL_EXTENT_MM      25.0f
#define G_MM_S2             9810.0f
#define MIN_AMP_DEG 1.0f

BallData ball_data;
unsigned long time_us;

// Amplitude (deg) that keeps ball travel near BALL_EXTENT_MM at this frequency.
float amp_for_freq(float freq)
{
    float w = 2.0f * PI * freq;
    float theta_rad = BALL_EXTENT_MM * w * w / (2.0f * (5.0f / 7.0f) * G_MM_S2);
    float theta_deg = theta_rad * 180.0f / PI;
    return constrain(theta_deg, MIN_AMP_DEG, MAX_ANGLE_DEVIATION);
}

//Generate a sinusoid of angles between Angle Min and Angle Max amplitude as a sin wave of a given freq
//Use the freq in Hz
double generate_cos_angle(float freq, int point, float amp) 
{
    return amp*cos(MIN_TSTEP*point*2*PI*freq);
}

bool wait_for_pc()
{
    while (!Serial.available()) {}              // block until the PC sends anything
    int byte=Serial.read();
    return byte == 0xC;
}

void run_sweep_X()
{
    for(int x=0; x<=(MAX_FREQ-MIN_FREQ)/FREQ_INCREMENT; x+=1)
    {
        float freq_x=MIN_FREQ+x*FREQ_INCREMENT;
        setTilt(0,0,&ball_data);
        while(!wait_for_pc()){} //start each frequency run
        int total_points = NUM_OSCILLATIONS/(freq_x*MIN_TSTEP)+0.5;
        float amp = amp_for_freq(freq_x);
        time_us=micros();
        for(int i =0; i<total_points;i++)
        {
            float angle_x=generate_cos_angle(freq_x,i,amp);
            setTilt(angle_x,0,&ball_data);
            time_us += (unsigned long)(MIN_TSTEP * 1e6f);   // 3. move the deadline forward by 50,000 us
            while ((long)(micros() - time_us) < 0) {}
        }
    }
}

void run_sweep_Y()
{
    for(int y=0; y<=(MAX_FREQ-MIN_FREQ)/FREQ_INCREMENT; y+=1)
    {
        float freq_y=MIN_FREQ+y*FREQ_INCREMENT;
        setTilt(0,0,&ball_data);
        while(!wait_for_pc()){} //start each frequency run
        //need to wait for ball confirmation here before continuing
        int total_points = NUM_OSCILLATIONS/(freq_y*MIN_TSTEP)+0.5;
        float amp = amp_for_freq(freq_y);
        time_us=micros();
        for(int i =0; i<total_points;i++)
        {
            float angle_y=generate_cos_angle(freq_y,i,amp);
            setTilt(0,angle_y,&ball_data);
            time_us += (unsigned long)(MIN_TSTEP * 1e6f);   // 3. move the deadline forward by 50,000 us
            while ((long)(micros() - time_us) < 0) {}
        }
    }
}




