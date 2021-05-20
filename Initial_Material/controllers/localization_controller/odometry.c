#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "odometry.h"

//-----------------------------------------------------------------------------------//
/*CONSTANTES*/
#define WHEEL_AXIS 0.057	// Distance between the two wheels in meter
#define WHEEL_RADIUS 0.0200 // Radius of the wheel in meter

/*VERBOSE_FLAGS*/
#define VERBOSE_ODO_ENC true // Print odometry values computed with wheel encoders
#define VERBOSE_ODO_ACC true // Print odometry values computed with accelerometer
//-----------------------------------------------------------------------------------//
/*GLOBAL*/
static double _T;

static pose_t _odo_pose_acc, _odo_speed_acc, _odo_pose_enc;
//-----------------------------------------------------------------------------------//

/**
 * @brief      Compute the odometry using the acceleration
 *
 * @param      odo       The odometry
 * @param[in]  acc       The acceleration
 * @param[in]  acc_mean  The acc mean
 */
void odo_compute_acc_encoders(pose_t *odo, const double acc[3], const double acc_mean[3], double Aleft_enc, double Aright_enc)
{
	// To Do : Compute the acceleration in frame A + remove biais (Assume 1-D motion)
	double acc_wx = acc[1] - acc_mean[1];
	double acc_wy = acc[0] - acc_mean[0];

	Aleft_enc = Aleft_enc * WHEEL_RADIUS * 2 * 3.14;

	Aright_enc = Aright_enc * WHEEL_RADIUS * 2 * 3.14;

	double omega_w = (Aright_enc - Aleft_enc) / (WHEEL_AXIS * _T);

	// To Do : Implement your motion model (Assume 1-D motion)
	_odo_speed_acc.x = _odo_speed_acc.x + acc_wx * _T;

	_odo_pose_acc.x = _odo_pose_acc.x + _odo_speed_acc.x * _T;

	_odo_speed_acc.y = _odo_speed_acc.y + acc_wy * _T;

	_odo_pose_acc.y = _odo_pose_acc.y + _odo_speed_acc.y * _T;

	_odo_pose_acc.heading = _odo_pose_acc.heading + omega_w * _T;

	memcpy(odo, &_odo_pose_acc, sizeof(pose_t));

	//printf("ODO with acceleration : %g %g %g\n", odo->x, odo->y, RAD2DEG(odo->heading));
	printf("ODO with accelertaion : speed_x: %g, speed_y: %g\n", _odo_speed_acc.x, _odo_speed_acc.y);
}

/**
 * @brief      Compute the odometry using the encoders
 *
 * @param      odo         The odometry
 * @param[in]  Aleft_enc   The delta left encoder
 * @param[in]  Aright_enc  The delta right encoder
 */
void odo_compute_encoders(pose_t *odo, double Aleft_enc, double Aright_enc)
{

	// Rad to meter
	Aleft_enc  *= WHEEL_RADIUS;

	Aright_enc *= WHEEL_RADIUS;

	// Compute forward speed and angular speed
	double omega = ( Aright_enc - Aleft_enc ) / ( WHEEL_AXIS * _T );

	double speed = ( Aright_enc + Aleft_enc ) / ( 2.0 * _T );

	// Apply rotation (Body to World)

	double a = _odo_pose_enc.heading;

	double speed_wx = speed * cos(a);

	double speed_wy = speed * sin(a);

	// Integration : Euler method
	_odo_pose_enc.x += speed_wx * _T;

	_odo_pose_enc.y += speed_wy * _T;

	_odo_pose_enc.heading += omega * _T;

	memcpy(odo, &_odo_pose_enc, sizeof(pose_t));

	memcpy(odo, &_odo_pose_enc, sizeof(pose_t));

	//printf("ODO with wheel encoders : %g %g %g\n", odo->x, odo->y, RAD2DEG(odo->heading));
	printf("ODO with encorder : speed_x: %g, speed_y: %g\n", speed_wx, speed_wy);
	printf("Aleft_enc is: %g, Aright_enc is: %g, speed is: %g\n", Aleft_enc, Aright_enc, speed);
}

/**
 * @brief      Reset the odometry to zeros
 *
 * @param[in]  time_step  The time step used in the simulation in miliseconds
 */
void odo_reset(int time_step)
{

	memset(&_odo_pose_acc, 0, sizeof(pose_t));

	memset(&_odo_speed_acc, 0, sizeof(pose_t));

	memset(&_odo_pose_enc, 0, sizeof(pose_t));

	_T = time_step / 1000.0;
}