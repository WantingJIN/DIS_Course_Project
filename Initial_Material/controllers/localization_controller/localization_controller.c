#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>

#include "trajectories.h"
#include "odometry.h"
#include <webots/emitter.h>
#include "kalman_filter.h"
//----------------------------------------------------------
/* FLAGS_ENABLE_DIFFERENT LOCALIZATION_METHOD*/
#define USE_ENCODER_ONLY true
#define USE_GPS_ONLY true
#define USE_ACCELEROMETER_ENCODER true
#define USE_KALMAN_FILTER true
//----------------------------------------------------------
/*DEFINITION*/
#define TIME_INIT_ACC 5 // Time in second
#define FLOCK_SIZE 1
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
//-----------------------------------------------------------
/* VARIABLES */
WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor;
WbDeviceTag dev_right_motor;
WbDeviceTag radio_emitter;
//WbDeviceTag infrared_emitter；

static measurement_t _meas;
// We have the following formulation of pose variable:  X= [pos_x, pos_y, heading]^T;
static pose_t _estimated_pose, _gps_pose, _odo_acc_encoder, _odo_enc;
// We have the following formulation of state variable:  X= [pos_x, pos_y, vel_x, vel_y]^T;
static state_t _kalman_state;
static pose_t _pose_origin = {-2.9, 0.0, 0.0};
double last_gps_time = 0.0f;
int time_step;
char *robot_name;
int robot_id_u, robot_id; // Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
//----------------------------------------------------------
/*FUNCTIONS*/
static void controller_init(int ts);
static void init_devices(int ts);
static void controller_get_pose();
static void controller_get_acc();
static void controller_get_encoder();
static double controller_get_heading();
static void controller_compute_mean_acc();

//-----------------------------------------------------------
void controller_init(int time_step)
{
  init_devices(time_step);

  memset(&_meas, 0, sizeof(measurement_t));

  memset(&_gps_pose, 0, sizeof(pose_t));

  memset(&_estimated_pose, 0, sizeof(pose_t));

  memset(&_odo_enc, 0, sizeof(pose_t));

  memset(&_odo_acc_encoder, 0, sizeof(pose_t));

  memset(&_kalman_state, 0, sizeof(state_t));

  odo_reset(time_step);

  kalman_filter_reset(time_step);
}
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

  radio_emitter = wb_robot_get_device("emitter");
  robot_name = (char *)wb_robot_get_name();
  sscanf(robot_name, "epuck%d", &robot_id_u); // read robot id from the robot's name
  robot_id = robot_id_u % FLOCK_SIZE;         // normalize between 0 and FLOCK_SIZE-1
}

void controller_get_gps()
{
  // To Do : store the previous measurements of the gps (use memcpy)
  memcpy(_meas.prev_gps, _meas.gps, sizeof(_meas.gps));
  // To Do : get the positions from webots for the gps. Uncomment and complete the following line Note : Use _robot.gps
  const double *gps_position = wb_gps_get_values(dev_gps);
  // To Do : Copy the gps_position into the measurment structure (use memcpy)
  memcpy(_meas.gps, gps_position, sizeof(_meas.gps));

  //printf("ROBOT gps is at position: %g %g %g\n", _meas.gps[0], _meas.gps[1], _meas.gps[2]);
}

void controller_get_encoder()
{
  // Store previous value of the left encoder
  _meas.prev_left_enc = _meas.left_enc;

  _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);

  // Store previous value of the right encoder
  _meas.prev_right_enc = _meas.right_enc;

  _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);

  //printf("ROBOT enc : %g %g\n", _meas.left_enc, _meas.right_enc);
}

void controller_get_acc()
{
  const double *acc_values = wb_accelerometer_get_values(dev_acc);

  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));

  //printf("ROBOT acc : %g %g %g\n", _meas.acc[0], _meas.acc[1], _meas.acc[2]);
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
    _gps_pose.x = _meas.gps[0] - _pose_origin.x;

    _gps_pose.y = -(_meas.gps[2] - _pose_origin.y);

    _gps_pose.heading = controller_get_heading() + _pose_origin.heading;

    //printf("ROBOT pose : %g %g %g\n", _gps_pose.x, _gps_pose.y, RAD2DEG(_gps_pose.heading));
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
void controller_compute_mean_acc()
{
  static int count = 0;

  count++;

  if (count > 20) // Remove the effects of strong acceleration at the begining
  {
    for (int i = 0; i < 3; i++)
      _meas.acc_mean[i] = (_meas.acc_mean[i] * (count - 21) + _meas.acc[i]) / (double)(count - 20);
  }

  if (count == (int)(TIME_INIT_ACC / (double)time_step * 1000))
  {
    printf("Accelerometer initialization Done ! \n");
    printf("acc_mean is: [0]%f, [1]%f \n", _meas.acc_mean[0], _meas.acc_mean[1]);
  }

  //printf("ROBOT acc mean : %g %g %g\n", _meas.acc_mean[0], _meas.acc_mean[1], _meas.acc_mean[2]);
}

//----------------------------------------------------------
/*MAIN FUNCTION*/
int main()
{
  char buffer[255]; // Buffer for sending data
  wb_robot_init();
  //time_step = wb_robot_get_basic_time_step();
  time_step = 64;
  controller_init(time_step);

  while (wb_robot_step(time_step) != -1)
  {

    // 1. Perception / Measurement
    controller_get_pose();

    controller_get_acc();

    controller_get_encoder();

    if (wb_robot_get_time() < TIME_INIT_ACC)
    {
      controller_compute_mean_acc();
      wb_motor_set_velocity(dev_left_motor, 0.0);
      wb_motor_set_velocity(dev_right_motor, 0.0);
    }
    else
    {
      if (USE_GPS_ONLY)
      {
        memcpy(&_estimated_pose, &_gps_pose, sizeof(pose_t));
      }
      if (USE_ACCELEROMETER_ENCODER)
      {
        odo_compute_acc_encoders(&_odo_acc_encoder, _meas.acc, _meas.acc_mean, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
        memcpy(&_estimated_pose, &_odo_acc_encoder, sizeof(pose_t));
      }
      if (USE_ENCODER_ONLY)
      {
        odo_compute_encoders(&_odo_enc, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
        memcpy(&_estimated_pose, &_odo_enc, sizeof(pose_t));
      }
      if (USE_KALMAN_FILTER)
      {
        kalman_filter_compute_pose(&_kalman_state, &_gps_pose, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
      }
      // Use one of the two trajectories.
      trajectory_2(dev_left_motor, dev_right_motor);
      //    trajectory_2(dev_left_motor, dev_right_motor);
    }

    // Send the estimated pose to supervisor

    sprintf(buffer, "%1d#%f#%f", robot_id, _estimated_pose.x, _estimated_pose.y);
    //sprintf(buffer, "%1d#%f#%f", 1, 1.2, 1.2);
    //printf("message sent: %s\n", buffer);
    wb_emitter_send(radio_emitter, buffer, strlen(buffer));
    //printf("Robot%d estimated_pose_x: %f y: %f\n", robot_id, _estimated_pose.x, _estimated_pose.y);
  }

  kalman_filter_cleanup();

  wb_robot_cleanup();

  return 0;
}