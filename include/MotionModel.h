/**
	CMU ROB-16831 Project 4
	MotionModel.h
	Purpose: Implement a motion model

	@author Miao Yu
	@version 1.0 11/26/2015
*/

#ifndef __MOTION_MODEL_H_
#define __MOTION_MODEL_H_

#include "constants.h"
#include "ParticleFilterType.h"
#include "Probability.h"

namespace PFL
{
	class MotionModel
	{
	public:

		/**
			Constructor of the class MotionModel

			@param None
			@return None
		*/
		MotionModel();

		/**
			Destructor of the class MotionModel

			@param None
			@return None
		*/
		~MotionModel();

		/**
			Sampling based on odometry motion model

			@param state The state before receiving odometry
			@param odomPrev The odometry data in the last timestep
			@param odomCurr The odometry data in the current timestep
			@return The updated particle state
		*/
		Particle sampling(Particle state, OdomData odomPrev, OdomData odomCurr);

	private:

		// An instance of class Probability 
		Probability helper;

	};
}


#endif