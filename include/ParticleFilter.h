/**
	CMU ROB-16831 Project 4
	ParticleFilter.h
	Purpose: Implement a particle filter

	@author Miao Yu
	@version 1.0 11/25/2015
*/
#ifndef __PARTICLE_FILTER_H_
#define __PARTICLE_FILTER_H_	

#include "ParticleFilterType.h"
#include "MotionModel.h"
#include "MeasurementModel.h"

namespace PFL
{
	class ParticleFilter
	{

	public:

		/**
			Constructor of the class ParticleFilter

			@param None
			@return None
		*/
		ParticleFilter();
		
		/**
			Constructor of the class ParticleFilter

			@param inputMap Input map
			@return None
		*/
		ParticleFilter(MapType inputMap);

		/**
			Destructor of the class ParticleFilter

			@param None
			@return None
		*/
		~ParticleFilter();

		/**
			Generate initial particles

			@param None
			@return samples All current particles
		*/
		std::vector<Particle> generateParticles();

		/** 
			Motion update

			@param odom Odometry data
			@return None
		*/
		std::vector<Particle> update(OdomData odomPrev, OdomData odomCurr);

		/** 
			Measurement update

			@param meas Measurement data
			@return None
		*/
		Particle update(MeasData meas);

	private:

		// All current particles
		std::vector<Particle> samples;

		// Map
		MapType map;

		// An instance of class MotionModel
		MotionModel *motionModel;


		// An instance of class MeasurementModel
		MeasurementModel *measurementModel;
		/** 
			Low variance resampling

			@param Nonr
			@return None
		*/
		void resampling();
	};
}

#endif
