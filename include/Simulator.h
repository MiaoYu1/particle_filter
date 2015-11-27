/**
	CMU ROB-16831 Project 4
	Simulator.h
	Purpose: A simulator which runs particle filter

	@author Miao Yu
	@version 1.0 11/25/2015
*/

#ifndef __SIMULATOR_H_
#define __SIMULATOR_H_

#include "ParticleFilterType.h"
#include "ParticleFilter.h"

namespace PFL
{

	class Simulator
	{

	public:

		/**
			Constructor of class Simulator

			@param inputMapName The input map file name
			@param inputLogName The input log file name		
			@return None
		*/
		Simulator(char *inputMapName, char *inputLogName);

		/**
			Destructor of class Simlator

			@param None
			@return None
		*/
		~Simulator();

		/**
			Read the given map of Wean Hall

			@param None
			@return 0 -> Successfully read a map
					1 -> Some error happened
		*/
		int readMap();

		/**
			Read the sensor log file to localize the mobile robot

			@param None
			@return 0 -> Successfully read a map
					1 -> Some error happened
		*/
		int readLog();

		/**
			Execute the log file

			@param none
			@return None
		*/
		void execution();

	private:

		// Name of the map file
		char *mapName;

		// Name of the log file
		char *logName;

		// An instance of type MapType
		MapType map;

		// An instance of type LogType
		LogType sensorLog;

		// And instance of class ParticleFilter
		ParticleFilter *pf;

		// All current particles
		std::vector<Particle> samples;

		// All current measurement 
		std::vector<MeasData> measurements;

		/**
			Parse the odometry data

			@param line One line of the log file
			@return Odometry data in OdomData type
		*/
		OdomData odomDataParse(char *line);
		
		/**
			Parse the measurement data

			@param line One line of the log file
			@return Measurement data in MeasData type
		*/
		MeasData measDataParse(char *line);
		
		/**
			Visualize the input map

			@param none
			@return none
		*/
		void drawMap();

		/**
			Visualize the current particles

			@param none
			@return none
		*/
		void drawParticles();

		/**
			Visualize the current measurement

			@param state The state with highest probability
			@param measData Current measurement
			@return none
		*/
		void drawLasers(Particle state, MeasData measData);

	};	

}

#endif
