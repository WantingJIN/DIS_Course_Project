

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE 1 // Number of robots in flock for localization world
// #define FLOCK_SIZE 5  //Number of robots in flock for obstacle world
#define TIME_STEP 64 // [ms] Length of time step

WbNodeRef robs[FLOCK_SIZE];			  // Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	  // Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE]; // Robots rotation fields
WbDeviceTag receiver;				  //Single recevier

float loc[FLOCK_SIZE][3];			 // Location of everybody in the flock
float estimated_pose[FLOCK_SIZE][2]; // Estimated position of each robot by using different localization method
float initial_loc[FLOCK_SIZE][3];	 // Initial localization of every epucks in the flock

#define RULE1_THRESHOLD 0.2
#define fit_cluster_ref 0.03
#define fit_orient_ref 1.0

int offset;			// Offset of robots number
float migrx, migrz; // Migration vector
float orient_migr;	// Migration orientation
int t;

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
	// int i;
	// // Load robot field for flocking
	// for (i = 0; i < FLOCK_SIZE; i++)
	// {
	// 	sprintf(rob, "Robot%d", i + offset);
	// 	robs[i] = wb_supervisor_node_get_from_def(rob);
	// 	robs_trans[i] = wb_supervisor_node_get_field(robs[i], "translation");
	// 	robs_rotation[i] = wb_supervisor_node_get_field(robs[i], "rotation");
	// }
	// Load robot field for single robot
	sprintf(rob, "ROBOT%d", 1);
	robs[0] = wb_supervisor_node_get_from_def(rob);
	robs_trans[0] = wb_supervisor_node_get_field(robs[0], "translation");
	robs_rotation[0] = wb_supervisor_node_get_field(robs[0], "rotation");
}
/*
 * Get the inital position of robots
 */

void get_robots_initial_position()
{
	int i;
	for (i = 0; i < FLOCK_SIZE; i++)
	{
		initial_loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];		  // X
		initial_loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];		  // Z
		initial_loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
	}
	// Run one step
	wb_robot_step(TIME_STEP);
}

/*
 * Compute localization metric
 */

void compute_localization_fitness(float *fit_loc)
{
	*fit_loc = 0;
	int i;
	for (i = 0; i < FLOCK_SIZE; i++)
	{
		*fit_loc += sqrt((powf(loc[i][0] - initial_loc[i][0] - estimated_pose[i][0], 2) + powf(-(loc[i][1] -initial_loc[i][1]) - estimated_pose[i][1], 2)));
	}
}

/*
 * Compute flocking performance metric.
 */
void compute_flocking_fitness(float *fit_c, float *fit_o)
{
	*fit_c = 0;
	*fit_o = 0;
	// Compute performance indices
	// Based on distance of the robots compared to the threshold and the deviation from the perfect angle towards
	// the migration goal
	float angle_diff;
	int i;
	int j;
	for (i = 0; i < FLOCK_SIZE; i++)
	{
		for (j = i + 1; j < FLOCK_SIZE; j++)
		{
			// Distance measure for each pair ob robots
			*fit_c += fabs(sqrtf(powf(loc[i][0] - loc[j][0], 2) + powf(loc[i][1] - loc[j][1], 2)) - RULE1_THRESHOLD * 2);
		}

		// Angle measure for each robot
		angle_diff = fabsf(loc[i][2] - orient_migr);
		*fit_o += angle_diff > M_PI ? 2 * M_PI - angle_diff : angle_diff;
	}
	*fit_c /= FLOCK_SIZE * (FLOCK_SIZE + 1) / 2;
	*fit_o /= FLOCK_SIZE;
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
	// float fit_cluster; // Performance metric for aggregation
	// float fit_orient;  // Performance metric for orientation
	float fit_localization; //Performance metric for localization
	get_robots_initial_position();
	for (;;)
	{
		wb_robot_step(TIME_STEP);

		int count = 0;
		while (wb_receiver_get_queue_length(receiver) > 0 && count < FLOCK_SIZE)
		{
			inbuffer = (char *)wb_receiver_get_data(receiver);
			sscanf(inbuffer, "%d#%f#%f", &robot_id, &rob_x, &rob_z);
			estimated_pose[robot_id][0] = rob_x;
			estimated_pose[robot_id][1] = rob_z;
			//printf("message receive: %s\n", inbuffer);
			//printf("Robot %d estimated pose is: %f %f\n", robot_id, estimated_pose[robot_id][0], estimated_pose[robot_id][1]);
			count++;
			wb_receiver_next_packet(receiver);
		}
		for (i = 0; i < FLOCK_SIZE; i++)
		{
			loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];		  // X
			loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];		  // Z
			loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
		}
		compute_localization_fitness(&fit_localization);
		//printf("fitness for localization is: %f \n", fit_localization);
		if (t % 10 == 0)
		{
			// compute_flocking_fitness(&fit_cluster, &fit_orient);
			// fit_cluster = fit_cluster_ref / fit_cluster;
			// fit_orient = 1 - fit_orient / M_PI;
		}
		t += TIME_STEP;
	}
}
