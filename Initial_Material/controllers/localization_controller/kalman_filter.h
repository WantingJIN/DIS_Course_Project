#include "odometry.h"

typedef struct
{
    double x;
    double y;
    double theta;
    double vx;
    double vy;
    double omega;
} state_t;

void kalman_filter_reset(int time_stamp);
void kalman_filter_compute_pose(state_t *estimate_state, pose_t *gps_pose, const double Aleft_enc, const double Aright_enc);
void kalman_filter_cleanup();