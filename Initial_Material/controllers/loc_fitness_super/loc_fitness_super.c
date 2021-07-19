#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

//----------------------------------------------------------
/*DEFINITION*/

// #define FLOCK_SIZE 1 // Number of robots in flock for localization world
#define FLOCK_SIZE 5 //Number of robots in flock for obstacle world
#define TIME_STEP 64 // [ms] Length of time step

#define MAX_SPEED_WEB 6.28 // Maximum speed webots
#define MAX_SPEED 800	   // Maximum speed
#define TARGET_FLOCKING_DISTANCE 0.14
#define WHEEL_RADIUS 0.0205
//----------------------------------------------------------
/*GLOBAL VARIABLE*/
WbNodeRef robs[FLOCK_SIZE];			  // Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	  // Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE]; // Robots rotation fields
WbDeviceTag receiver;				  //Single recevier

float loc[FLOCK_SIZE][3];			 // Location of everybody in the flock
float estimated_pose[FLOCK_SIZE][2]; // Estimated position of each robot by using different localization method
int offset;							 // Offset of robots number
float migrx, migrz;					 // Migration vector
float orient_migr;					 // Migration orientation
int t;
float prev_flocking_center[2];

/*
 * Initialize flock position and devices
 */
void reset(void)
{
	wb_robot_init();

	receiver = wb_robot_get_device("receiver");
	wb_receiver_enable(receiver, 1);

	wb_receiver_next_packet(receiver);

	if (receiver == 0)
		printf("missing receiver\n");

	char rob[7] = "epuck0";
	int i;
	// Load robot field for flocking
	for (i = 0; i < FLOCK_SIZE; i++)
	{
		sprintf(rob, "epuck%d", i + offset);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i], "translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i], "rotation");
	}
	// Load robot field for single robot
	//sprintf(rob, "ROBOT%d", 1);
	// robs[0] = wb_supervisor_node_get_from_def(rob);
	// robs_trans[0] = wb_supervisor_node_get_field(robs[0], "translation");
	// robs_rotation[0] = wb_supervisor_node_get_field(robs[0], "rotation");
}
/*
 * Get Initial Flocking Center
 */
void get_initial_flocking_center()
{
	int i;
	for (i = 0; i < FLOCK_SIZE; i++)
	{
		loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];		  // X
		loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];		  // Z
		loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
		prev_flocking_center[0] += loc[i][0];
		prev_flocking_center[1] += loc[i][1];
	}
	prev_flocking_center[0] /= FLOCK_SIZE;
	prev_flocking_center[1] /= FLOCK_SIZE;
}
/*
 * Compute localization metric
 */

void compute_localization_fitness(float *fit_loc)
{
	*fit_loc = 0;
	int i;
	// When calculated error between the estimated pose and the pose get from webot world, covert y axis
	for (i = 0; i < FLOCK_SIZE; i++)
	{
		*fit_loc += sqrt((powf(loc[i][0] - estimated_pose[i][0], 2) + powf(-loc[i][1] - estimated_pose[i][1], 2)));
	}
}

/*
 * Compute flocking performance metric.
 */
void compute_flocking_fitness(float *fit_flocking)
{
	float fit_dist = 0.0;
	float fit_heading = 0.0;
	float fit_vel_towards_goal = 0.0;
	// Compute performance indices
	// Based on distance of the robots compared to the threshold and the deviation from the perfect angle towards
	// the migration goal
	float fit_inter_dist = 0.0;
	float fit_flocking_center_dist = 0.0;
	float flocking_center[] = {0.0, 0.0};
	float dist_diff;
	float N_pairs = FLOCK_SIZE * (FLOCK_SIZE + 1) / 2;
	float speed_max = MAX_SPEED_WEB * MAX_SPEED * WHEEL_RADIUS / 1000;
	float Dmax = speed_max * TIME_STEP;
	int i;
	int j;
	for (i = 0; i < FLOCK_SIZE; i++)
	{
		flocking_center[0] += loc[i][0];
		flocking_center[1] += loc[i][1];
	}
	flocking_center[0] /= FLOCK_SIZE;
	flocking_center[1] /= FLOCK_SIZE;

	for (i = 0; i < FLOCK_SIZE; i++)
	{
		for (j = i + 1; j < FLOCK_SIZE; j++)
		{
			// Distance measure for each pair of robots
			dist_diff = fabs(sqrtf(powf(loc[i][0] - loc[j][0], 2) + powf(loc[i][1] - loc[j][1], 2)));
			fit_inter_dist += fmin(dist_diff / TARGET_FLOCKING_DISTANCE, 1 / powf(1 - TARGET_FLOCKING_DISTANCE + dist_diff, 2));
			// Heading angle measure for each pair of robots
			fit_heading += fabs(loc[i][2] - loc[j][2]) / M_PI;
		}

		// Distance measure between each robot and flocking center
		fit_flocking_center_dist += fabs(sqrtf(powf(loc[i][0] - flocking_center[0], 2) + powf(loc[i][1] - flocking_center[1], 2)));
	}

	fit_inter_dist /= N_pairs;
	fit_flocking_center_dist /= FLOCK_SIZE;
	fit_dist = 1 / (1 + fit_flocking_center_dist) * fit_inter_dist;
	fit_heading /= N_pairs;
	fit_heading = 1 - fit_heading;

	fit_vel_towards_goal = fabs(sqrtf(powf(flocking_center[0] - prev_flocking_center[0], 2) + powf(flocking_center[1] - prev_flocking_center[1], 2))) / (Dmax / 1000);
	*fit_flocking = fit_heading * fit_dist * fit_vel_towards_goal;
	//printf("fitness for flocking is: %f, %f, %f\n", fit_heading, fit_dist, fit_vel_towards_goal);
	prev_flocking_center[0] = flocking_center[0];
	prev_flocking_center[1] = flocking_center[1];
}

/*
 * Main function.
 */

int main(int argc, char *args[])
{
	char *inbuffer;
	int robot_id, i;
	float rob_x, rob_z;
	reset();
	float fit_flocking;
	float fit_localization; //Performance metric for localization
	bool recevied_loc_data = false;
	get_initial_flocking_center();
	for (;;)
	{
		wb_robot_step(TIME_STEP);

		int count = 0;
		while (wb_receiver_get_queue_length(receiver) > 0 && count < FLOCK_SIZE)
		{
			recevied_loc_data = true;
			inbuffer = (char *)wb_receiver_get_data(receiver);
			sscanf(inbuffer, "%d#%f#%f", &robot_id, &rob_x, &rob_z);
			estimated_pose[robot_id][0] = rob_x;
			estimated_pose[robot_id][1] = rob_z;
			//printf("message receive: %s\n", inbuffer);
			//printf("Robot %d estimated pose is: %f %f\n", robot_id, estimated_pose[robot_id][0], estimated_pose[robot_id][1]);
			count++;
			//printf("Recevied message from robot %d: %s\n", robot_id, inbuffer);
			wb_receiver_next_packet(receiver);
		}
		for (i = 0; i < FLOCK_SIZE; i++)
		{
			loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];		  // X
			loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];		  // Z
			loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
		}
		if (recevied_loc_data)
		{
			compute_localization_fitness(&fit_localization);
			printf("fitness for localization is: %f \n", fit_localization);
		}

		//compute_flocking_fitness(&fit_flocking);
		//printf("fitness for flocking is: %f \n", fit_flocking);

		// TODO: set a proper stop condition and calculate the average value of the flocking fitness
	}
}
