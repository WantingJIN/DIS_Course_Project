#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>

#include "trajectories.h"

//----------------------------------------------------------
/* FLAGS_ENABLE_DIFFERENT LOCALIZATION_METHOD*/
#define USE_ENCODER_ONLY false
#define USE_GPS_ONLY true
#define USE_ACCELEROMETER_ENCODER false
#define USE_KALMAN_FILTER false
//----------------------------------------------------------
/*DEFINITION*/
#define RAD2DEG(X)      X / M_PI * 180.0
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
typedef struct 
{
  double x;
  double y;
  double heading;
} pose_t;
//-----------------------------------------------------------
/* VARIABLES */
WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor;
WbDeviceTag dev_right_motor;

static measurement_t _meas;
static pose_t _pose, _odo_acc, _odo_enc;
static pose_t _pose_origin = {-0.4, 0.4, 0.0};
double last_gps_time = 0.0f;

//----------------------------------------------------------
/*FUNCTIONS*/
void init_devices(int ts);
static void controller_get_pose();
//static void controller_get_acc();
//static void controller_get_encoder();
static double controller_get_heading();

//----------------------------------------------------------
/*MAIN FUNCTION*/
int main()
{
  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  init_devices(time_step);

  while (wb_robot_step(time_step) != -1)
  {
    // 1. Perception / Measurement
    controller_get_pose();

    //controller_get_acc();

    //controller_get_encoder();

    // Use one of the two trajectories.
    trajectory_1(dev_left_motor, dev_right_motor);
    //    trajectory_2(dev_left_motor, dev_right_motor);
  }
}

//-----------------------------------------------------------

void init_devices(int ts)
{
  dev_gps = wb_robot_get_device("gps");
  wb_gps_enable(dev_gps, 1000);

  dev_acc = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(dev_acc, ts);

  dev_left_encoder = wb_robot_get_device("left wheel sensor");
  dev_right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(dev_left_encoder, ts);
  wb_position_sensor_enable(dev_right_encoder, ts);

  dev_left_motor = wb_robot_get_device("left wheel motor");
  dev_right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(dev_left_motor, INFINITY);
  wb_motor_set_position(dev_right_motor, INFINITY);
  wb_motor_set_velocity(dev_left_motor, 0.0);
  wb_motor_set_velocity(dev_right_motor, 0.0);
}

void controller_get_gps()
{
  // To Do : store the previous measurements of the gps (use memcpy)
  memcpy(_meas.prev_gps, _meas.gps, sizeof(_meas.gps));
  // To Do : get the positions from webots for the gps. Uncomment and complete the following line Note : Use _robot.gps
  const double *gps_position = wb_gps_get_values(dev_gps);
  // To Do : Copy the gps_position into the measurment structure (use memcpy)
  memcpy(_meas.gps, gps_position, sizeof(_meas.gps));

  printf("ROBOT gps is at position: %g %g %g\n", _meas.gps[0], _meas.gps[1], _meas.gps[2]);
}

void controller_get_pose()
{
  // Call the function to get the gps measurements
  double time_now_s = wb_robot_get_time();
  if (time_now_s - last_gps_time > 1.0f)
  {
    last_gps_time = time_now_s;
    controller_get_gps();

    // To Do : Fill the structure pose_t {x, y, heading}. Use the _pose_origin.
    _pose.x = _meas.gps[0] - _pose_origin.x;

    _pose.y = -(_meas.gps[2] - _pose_origin.y);

    _pose.heading = controller_get_heading() + _pose_origin.heading;

     printf("ROBOT pose : %g %g %g\n", _pose.x, _pose.y, RAD2DEG(_pose.heading));
  }
}

double controller_get_heading()
{
  // To Do : implement your function for the orientation of the robot. Be carefull with the sign of axis y !
  double delta_x = _meas.gps[0] - _meas.prev_gps[0];

  double delta_y = -(_meas.gps[2] - _meas.prev_gps[2]);

  // To Do : compute the heading of the robot ( use atan2 )

  double heading = atan2(delta_y, delta_x);

  return heading;
}