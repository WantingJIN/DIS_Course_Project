#include "odometry.h"
#include <stdbool.h>

typedef struct
{
    double x;
    double y;
    double theta;
    double vx;
    double vy;
    double omega;
} state_t;

void kalman_filter_reset(int time_stamp, pose_t *pose_origin, int robot_id);
void kalman_filter_compute_pose(pose_t *state_kalman, pose_t *gps_pose, bool gps_updated, const double Aleft_enc, const double Aright_enc);
void kalman_filter_cleanup();