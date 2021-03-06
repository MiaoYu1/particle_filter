/**
	CMU ROB-16831 Project 4
	Constants.h
	Purpose: Define all constants

	@author Miao Yu
	@version 1.0 11/26/2015
*/

#ifndef __CONSTANTS_H_
#define __CONSTANTS_H_

#include <cmath>

// The number of range readings of laser 
#define NUM_RANGE 180
#define MAX_LASER 4000
#define LASER_OFFSET_X 2.5 // pixels
#define LASER_OFFSET_Y 0 // pixels

#define DEGREE2RADIAN M_PI/180

// Initial number of particles
#define NUM_PARTICLE 40000

// Motion model noise weights
#define alpha1 0.001
#define alpha2 0.1
#define alpha3 0.1
#define alpha4 0.001

// Measurement model noise weights
#define zHit 10
#define zRand 1
#define sigmaHit 6


#endif 
