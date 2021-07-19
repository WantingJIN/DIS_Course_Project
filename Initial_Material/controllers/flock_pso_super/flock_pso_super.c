#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pso.h>

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

//---------------------------------------------------------
/* FLAGS */
#define PSO_OPTIMIZATION true

//----------------------------------------------------------
/*DEFINITION*/

// #define FLOCK_SIZE 1 // Number of robots in flock for localization world
#define FLOCK_SIZE 5 //Number of robots in flock for obstacle world
#define TIME_STEP 64 // [ms] Length of time step

#define MAX_SPEED_WEB 6.28 // Maximum speed webots
#define MAX_SPEED 800	   // Maximum speed
#define TARGET_FLOCKING_DISTANCE 0.14
#define WHEEL_RADIUS 0.0205

//---------------------------------------------------------
#define ROBOTS 1
#define MAX_ROB 1
#define ROB_RAD 0.035
#define ARENA_LENGTH 5.8
#define ARENA_WIDTH 3.5
#define BRICK_NUM 10

//#define NB_SENSOR 8                     // Number of proximity sensors

/* PSO definitions */
//#define SWARMSIZE 10                    // Number of particles in swarm (defined in pso.h)
#define NB 1		 // Number of neighbors on each side
#define LWEIGHT 2.0	 // Weight of attraction to personal best
#define NBWEIGHT 2.0 // Weight of attraction to neighborhood best
#define VMAX 0.5	 // Maximum velocity particle can attain
#define MININIT 0.0	 // Lower bound on initialization value
#define MAXINIT 1.0	 // Upper bound on initialization value
#define ITS 100		 // Number of iterations to run
//#define DATASIZE 2*(NB_SENSOR+2+1)      // Number of elements in particle (2 Neurons with 8 proximity sensors
// + 2 recursive/lateral conenctions + 1 bias)
// defined in pso.h

/* Neighborhood types */
#define STANDARD -1
#define RAND_NB 0
#define NCLOSE_NB 1
#define FIXEDRAD_NB 2

/* Fitness definitions */
#define FIT_ITS 1800 // Number of fitness steps to run during optimization

#define FINALRUNS 10
#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8

//----------------------------------------------------------
/*GLOBAL VARIABLE*/
WbNodeRef robs[FLOCK_SIZE]; // Robots nodes
WbNodeRef bricks[BRICK_NUM];
WbFieldRef robs_trans[FLOCK_SIZE];	  // Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE]; // Robots rotation fields
WbFieldRef bricks_trans[BRICK_NUM];	  // Robots translation fields
WbDeviceTag receiver;				  //Single recevier
WbDeviceTag emitter[MAX_ROB];		  //emitter for different robot in case of hetegenous problem
WbDeviceTag emitter_loc;

float loc[FLOCK_SIZE][3];			 // Location of everybody in the flock
double initial_loc[FLOCK_SIZE][3];	 // Initial translation of everybody in the flock
double initial_rot[FLOCK_SIZE][4];	 // Initial rotation of everybody in the flock
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
	int i;
	wb_robot_init();

	receiver = wb_robot_get_device("receiver");
	wb_receiver_enable(receiver, 1);

	char em[] = "emitter0";
	for (i = 0; i < MAX_ROB; i++)
	{
		emitter[i] = wb_robot_get_device(em);
		em[7]++;
	}

	emitter_loc = wb_robot_get_device("emitter_loc");
	wb_receiver_next_packet(receiver);

	if (receiver == 0)
		printf("missing receiver\n");

	char rob[7] = "epuck0";
	// Load robot field for flocking
	for (i = 0; i < FLOCK_SIZE; i++)
	{
		sprintf(rob, "epuck%d", i + offset);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i], "translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i], "rotation");
	}
	char bri[7] = "brick0";
	for (i = 0; i < BRICK_NUM; i++)
	{
		sprintf(bri, "brick%d", i);
		bricks[i] = wb_supervisor_node_get_from_def(bri);
		bricks_trans[i] = wb_supervisor_node_get_field(bricks[i], "translation");
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
	int i, j;
	for (i = 0; i < FLOCK_SIZE; i++)
	{
		loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];		  // X
		loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];		  // Z
		loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
		prev_flocking_center[0] += loc[i][0];
		prev_flocking_center[1] += loc[i][1];

		memcpy(&initial_rot[i], wb_supervisor_field_get_sf_rotation(robs_rotation[i]), 4);
		for (j = 0; j < 3; j++)
		{
			initial_loc[i][j] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[j];
			initial_rot[i][j] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[j];
		}
		initial_rot[i][3] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3];
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

void calc_fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int its, int numRobs)
{
	double buffer[255];
	char loc_buffer[255];
	int i, j, t;
	float fit_flocking;
	printf("enter calculate fitness.\n");
	double sum_fitness = 0.0;
	/* Reset robots to initial position*/
	double zero_velocity[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	for (i = 0; i < FLOCK_SIZE; i++)
	{
		wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[i], "translation"), initial_loc[i]);
		wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[i], "rotation"), initial_rot[i]);
		wb_supervisor_node_set_velocity(robs[i], zero_velocity);
	}

	/* Randomlize the position of bricks */
	double new_brick_pos[3];
	for (i = 0; i < BRICK_NUM; i++)
	{
		new_brick_pos[0] = ARENA_LENGTH * rnd() - ARENA_LENGTH / 2;
		new_brick_pos[1] = 0.0;
		new_brick_pos[2] = ARENA_WIDTH * rnd() - ARENA_WIDTH / 2;
		wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(bricks[i], "translation"), new_brick_pos);
	}
	/* Send data to robots */
	for (i = 0; i < numRobs; i++)
	{
		for (j = 0; j < DATASIZE; j++)
		{
			buffer[j] = weights[i][j];
		}
		buffer[DATASIZE] = its; // set number of iterations at end of buffer
		wb_emitter_send(emitter[i], (void *)buffer, (DATASIZE + 1) * sizeof(double));
	}

	for (t = 0; t < its; t++)
	{
		wb_robot_step(TIME_STEP);
		for (i = 0; i < FLOCK_SIZE; i++)
		{
			loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];		  // X
			loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];		  // Z
			loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
			// Sending positions to the robots, comment the following two lines if you don't want the supervisor sending it
			sprintf(loc_buffer, "%1d#%f#%f#%f", i, loc[i][0], loc[i][1], loc[i][2]);
			wb_emitter_send(emitter_loc, loc_buffer, strlen(loc_buffer));
		}
		compute_flocking_fitness(&fit_flocking);
		sum_fitness += fit_flocking;
	}

	for (i = 0; i < numRobs; i++)
	{
		fit[i] = sum_fitness / its;
		printf("fitness is: %f\n", fit[i]);
	}
}

/*
Fitness function for PSO optimazation
*/

void fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int neighbors[SWARMSIZE][SWARMSIZE])
{

	calc_fitness(weights, fit, FIT_ITS, ROBOTS);

#if NEIGHBORHOOD == RAND_NB
	nRandom(neighbors, 2 * NB);
#endif
#if NEIGHBORHOOD == NCLOSE_NB
	nClosest(neighbors, 2 * NB);
#endif
#if NEIGHBORHOOD == FIXEDRAD_NB
	fixedRadius(neighbors, RADIUS);
#endif
}

/* Get distance between robots */
double robdist(int i, int j)
{
	return sqrt(pow(loc[i][0] - loc[j][0], 2) + pow(loc[i][2] - loc[j][2], 2));
}

/* Choose n random neighbors */
void nRandom(int neighbors[SWARMSIZE][SWARMSIZE], int numNB)
{

	int i, j;

	/* Get neighbors for each robot */
	for (i = 0; i < ROBOTS; i++)
	{

		/* Clear old neighbors */
		for (j = 0; j < ROBOTS; j++)
			neighbors[i][j] = 0;

		/* Set new neighbors randomly */
		for (j = 0; j < numNB; j++)
			neighbors[i][(int)(SWARMSIZE * rnd())] = 1;
	}
}
/*
 * Main function.
 */

int main(int argc, char *args[])
{
	double *flocking_weights;
	//char *inbuffer;
	wb_robot_init();
	printf("Particle Swarm Optimization Super Controller\n");
	reset();

	double buffer[255]; // Buffer for emitter
	int i, j, k;		// Counter variables
	// float fit_flocking;
	// float fit_localization; //Performance metric for localization
	// bool recevied_loc_data = false;
	get_initial_flocking_center();

	double fit;					 // Fitness of the current FINALRUN
	double endfit;				 // Best fitness over 10 runs
	double w[MAX_ROB][DATASIZE]; // Weights to be send to robots (determined by pso() )
	double f[MAX_ROB];			 // Evaluated fitness (modified by calc_fitness() )
	double bestfit, bestw[DATASIZE];

	/* Evolve controllers */
	endfit = 0.0;
	bestfit = 0.0;

	for (i = 0; i < 10; i++)
	{
		flocking_weights = pso(SWARMSIZE, NB, LWEIGHT, NBWEIGHT, VMAX, MININIT, MAXINIT, ITS, DATASIZE, ROBOTS);
		fit = 0.0;
		for (i = 0; i < MAX_ROB; i++)
		{
			for (k = 0; k < DATASIZE; k++)
				w[i][k] = flocking_weights[k];
		}

		// Run FINALRUN tests and calculate average

		calc_fitness(w, f, FIT_ITS, MAX_ROB);

		fit /= f[0];
		// Check for new best fitness
		if (fit > bestfit)
		{
			bestfit = fit;
			for (i = 0; i < DATASIZE; i++)
			{
				bestw[i] = flocking_weights[i];
			}
		}

		printf("Performance of the best solution: %.3f\n", fit);
		endfit += fit / 10; // average over the 10 runs
	}
	printf("~~~~~~~~ Optimization finished.\n");
	printf("Best performance: %.3f\n", bestfit);
	printf("Average performance: %.3f\n", endfit);

	/* Send best controller to robots */
	for (j = 0; j < DATASIZE; j++)
	{
		buffer[j] = bestw[j];
	}
	buffer[DATASIZE] = 1000000;
	for (i = 0; i < ROBOTS; i++)
	{
		wb_emitter_send(emitter[i], (void *)buffer, (DATASIZE + 1) * sizeof(double));
	}

	/* Wait forever */
	while (1)
		wb_robot_step(64);

	return 0;
}
