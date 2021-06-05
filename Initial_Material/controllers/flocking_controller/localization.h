#include "odometry.h"
#include "kalman_filter.h"

typedef struct
{
    double prev_gps[3];
    double gps[3];
    double acc_mean[3];
    double acc[3];
    double prev_left_enc;
    double left_enc;
    double prev_right_enc;
    double right_enc;
} measurement_t;
void localization_init(int ts);
void init_localization_devices(int ts);
void controller_get_pose();
void controller_get_acc();
void controller_get_encoder();
double controller_get_heading();
void controller_compute_mean_acc();
void estimate_self_position(float *estimate_pose, int localization_method);