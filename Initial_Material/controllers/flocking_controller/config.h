#define USE_GPS_ONLY false
#define USE_ENCODER_ONLY true
#define USE_ACCELEROMETER_ENCODER false
#define USE_KALMAN_FILTER false

/*LOCALIZATION INIT DEFINITION*/
#define TIME_INIT_ACC 5 // Time in second

//------------------------------------------------------------
/* Definition */
#define NB_SENSORS 8  // Number of distance sensors
#define MIN_SENS 350  // Minimum sensibility value
#define MAX_SENS 4096 // Maximum sensibility value
#define MAX_SPEED 800 // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB 6.28 // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE 5 // Size of flock
#define TIME_STEP 64 // [ms] Length of time step

#define AXLE_LENGTH 0.052       // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS 0.0205     // Wheel radius (meters)
#define DELTA_T 0.064           // Timestep (seconds)

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x >= 0) ? (x) : -(x))

#define DATASIZE 5

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

//--------------------------------------------------------------
/* Device Tag */
/*Webots 2018b*/
WbDeviceTag left_motor;  //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/
WbDeviceTag ds[NB_SENSORS];    // Handle for the infrared distance sensors
WbDeviceTag receiver_infrared; // Handle for the receiver node
WbDeviceTag emitter_infrared;  // Handle for the emitter node
WbDeviceTag receiver_radio;    // Handle for the recevier node
WbDeviceTag emitter_radio;     // Handle for the emitter node
WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;

measurement_t _meas;
// We have the following formulation of pose variable:  X= [pos_x, pos_y, heading]^T;
pose_t _estimated_pose, _gps_pose, _odo_acc_encoder, _odo_enc, _kalman_pose;
// We have the following formulation of state variable:  X= [pos_x, pos_y, vel_x, vel_y]^T;
pose_t _pose_origin = {-2.9, 0.0, 0.0};
double last_gps_time = 0.0f;
int time_step;
char *robot_name;
int robot_id_u, robot_id; // Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
bool gps_updated;