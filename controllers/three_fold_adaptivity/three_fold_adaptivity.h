/*
 * lifetimelearning.h
 *
 *  Created on: Nov 18, 2014
 *      Author: Jacqueline Heinerman
 */

#ifndef THREE_FOLD_ADAPTIVITY_H_
#define THREE_FOLD_ADAPTIVITY_H_

// Included libraries
#include <webots/robot.h> //obtain main library of webots
#include <webots/differential_wheels.h>   //obtain dif. wheels library
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <stdlib.h> //for abs
#include <math.h> // math functions
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <getopt.h>
#include <unistd.h>

// Global defines
#define NB_DIST_SENS 8		// number of ir sensors
#define LEFT 0        		// Left side
#define RIGHT 1       		// right side
#define INCR 10
#define TIME_STEP 50 		// [ms] // time step of the simulation
#define RANDOM_INIT_SIZE 5              // start with one of 10 best individuals
#define NMBRWEIGHTS 18		// number of weights in the network
#define MAXSPEED 500		// motor max speed
#define SENSOR_MAX 2500		// max ir sensor value

/*#define M_E		2.7182818284590452354*/
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// struct of candidate
typedef struct candidate {
	double weights[NMBRWEIGHTS];
	double fitness;
	double sigma;
	int genome;
} candidate;

typedef struct RobotPhenotypeDataMessage{
    char Destination[4];
    double Fitness;
    double Phenotype[NMBRWEIGHTS];
} RobotPhenotypeDataMessage;

typedef struct RobotGenotypeDataMessage{
  char Destination[4];
  double Fitness;
  int Genotype;
} RobotGenotypeDataMessage;

typedef struct Config
{
    float disable_sensor;
    int evolution;
    int sociallearning;
    float threshold;
    int total_evals;
    int max_robot_lifetime;
    int genome_tournament_size;
    float mutate_sensor;
    char experimentname[10];
} Configuration;




static void init();
static double* CreateRandomGenome();
static void Mutate(candidate* solution);
//static void MutateRandom(candidate* solution);
static double RunAndEvaluate(candidate* evaluee);
static double RunAndEvaluateForOneTimeStep(candidate* evaluee);
static double gaussrand();
static double mindouble(double a, double b);
static void SendGenome(candidate chmp, WbDeviceTag emitter);
bool parseParameters(int argc, char **argv);


#endif /* LIFETIMELEARNING_H_ */
