/**
	CMU ROB-16831 Project 4
	ParticleFilterType.h
	Purpose: Define all needed types

	@author Miao Yu
	@version 1.0 11/25/2015
*/
#ifndef __PARTICLE_FILTER_TYPE_H_
#define __PARTICLE_FILTER_TYPE_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include "constants.h"

namespace PFL
{
	struct MapType
	{
		//  Resolution of the map
		int resolution;

		// Offset on x and y axis
		float offsetX;
		float offsetY;

		// Size of the map
		int sizeX;
		int sizeY;

		// Boundary of the map 
		int minX;
		int maxX;
		int minY;
		int maxY;

		// Probability of occupancy
		cv::Mat prob;
	};

	struct LogType
	{
		// Total number of log
		// Attention: use long long int if neccesary
		int size;

		// Vector of log
		std::vector<std::string> log;
	};

	struct OdomData
	{
		// Coordinates of the robot in standard odometry frame
		double x;
		double y;
		double theta;

		// Timestamp of odometry reading
		double ts;
	};

	struct MeasData
	{
		// Coordinates of the robot in standard odometry frame
		double x;
		double y;
		double theta;

		// Coordinates of the laser in standard odometry frame
		double xLaser;
		double yLaser;
		double thetaLaser;

		// 180 range readings of laser in cm, in counterclockwise order
		int range[NUM_RANGE];

		// Timestamp of odometry reading
		double ts;
	};

	struct Particle
	{
		// The position and orientation of the robot
		double x;
		double y;
		double theta;
		double w;
	};
}

#endif