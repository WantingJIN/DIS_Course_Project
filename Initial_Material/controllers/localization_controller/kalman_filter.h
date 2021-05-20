#include "odometry.h"

typedef struct
{
    double x;
    double y;
    double vx;
    double vy;
} state_t;

void kalman_filter_reset(int time_stamp);
void kalman_filter_compute_pose(state_t *estimate_state, pose_t *gps_pose, const double acc[3], const double acc_mean[3]);
void kalman_filter_cleanup();