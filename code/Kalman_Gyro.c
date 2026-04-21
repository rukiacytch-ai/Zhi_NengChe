#include "zf_common_headfile.h"

static float yaw_unwrap(float new_yaw, float old_yaw_filtered)
{
    while (new_yaw < 0.0f) {
        new_yaw += 360.0f;
    }
    while (new_yaw >= 360.0f) {
        new_yaw -= 360.0f;
    }
    
    float old_yaw_normalized = fmod(old_yaw_filtered, 360.0f);
    if (old_yaw_normalized < 0.0f) {
        old_yaw_normalized += 360.0f;
    }
    
    float delta = new_yaw - old_yaw_normalized;
    
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }
    
    return old_yaw_filtered + delta;
}


Kalman_Filter_t kf_yaw = {
    .K = 0.0,
    .X = 179,
    .P = 0.0,
    .Q = 0.0,
    .R = 0.0,
};

void Yaw_Kalman_Filter_Init(float Q,float R)
{
    kf_yaw.K = 0.0,
    kf_yaw.X = 180.0,
    kf_yaw.P = 1.0,
    kf_yaw.Q = Q,
    kf_yaw.R = R;
}

float Kalman_Filter_Yaw_Update(float yaw_raw)
{

    float yaw_unwrapped = yaw_unwrap(yaw_raw, kf_yaw.X);

    kf_yaw.P += kf_yaw.Q;

    kf_yaw.K = kf_yaw.P / (kf_yaw.P + kf_yaw.R);
    kf_yaw.X = kf_yaw.X + kf_yaw.K * (yaw_unwrapped - kf_yaw.X);
    kf_yaw.P = (1 - kf_yaw.K) * kf_yaw.P;

    return kf_yaw.X;
}








