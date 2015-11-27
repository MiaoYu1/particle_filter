/**
	CMU ROB-16831 Project 4
	MeasurementModel.h
	Purpose: Implement a measurement model

	@author Miao Yu
	@version 1.0 11/26/2015
*/

#ifndef __MEASUREMENT_MODEL_H_
#define __MEASUREMENT_MODEL_H_

#include "ParticleFilterType.h"
#include "Probability.h"

namespace PFL
{
	class MeasurementModel
	{
	public:

		/**
			Constructor of the class MeasurementModel

			@param None
			@return None
		*/
		MeasurementModel();

		/**
			Constructor of the class MeasurementModel

			@param map The input map
			@return None
		*/
		MeasurementModel(MapType inputMap);

		/**
			Destructor of the class MeasurementModel

			@param None
			@return None
		*/
		~MeasurementModel();

		/**
			Update particle weight according to measurement

			@param measData The measurement of current state
			@return Probability based on the measurement model
		*/
		double weightUpdate(Particle state, MeasData measData);

	private:
		// Map
		MapType map;

		// An instance of class Probability 
		Probability helper;
	};
}

#endif