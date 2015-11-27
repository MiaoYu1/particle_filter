/**
	CMU ROB-16831 Project 4
	MeasurementModel.cpp
	Purpose: Implement a measurement model

	@author Miao Yu
	@version 1.0 11/26/2015
*/

#include "MeasurementModel.h"
#include "constants.h"
#include <cmath>

namespace PFL
{
	MeasurementModel::MeasurementModel(MapType inputMap)
	{
		map = inputMap;
	}

	MeasurementModel::~MeasurementModel(){}

	double MeasurementModel::weightUpdate(Particle state, MeasData measData)
	{
		double q = 0;
		for (int i = 0; i < NUM_RANGE; i+=3)
		{
			double minOccupied;
			if (measData.range[i] < MAX_LASER)
			{
				double xMeas = state.x + LASER_OFFSET_X * cos(state.theta) -
					LASER_OFFSET_Y * sin(state.theta) + (double)measData.range[i]/map.resolution * 
					cos(state.theta + (double)(i-90) * DEGREE2RADIAN);
				double yMeas = state.y + LASER_OFFSET_X * sin(state.theta) +
					LASER_OFFSET_Y * cos(state.theta) + (double)measData.range[i]/map.resolution * 
					sin(state.theta + (double)(i-90) * DEGREE2RADIAN);

				for (double j = 0; j < (double)MAX_LASER/map.resolution; j += 2)
				{
					int xMap = state.x + (int)(LASER_OFFSET_X * 
						cos(state.theta) - LASER_OFFSET_Y * sin(state.theta) + 
						j *	cos(state.theta + (double)(i-90) * DEGREE2RADIAN));
					int yMap = state.y + (int)(LASER_OFFSET_X * 
						sin(state.theta) + LASER_OFFSET_Y * cos(state.theta) + 
						j * sin(state.theta + (double)(i-90) * DEGREE2RADIAN));
					if (xMap <= map.maxX && xMap >= map.minX &&
						yMap <= map.maxY && yMap >= map.minY &&
						map.prob.at<float>(xMap,yMap) < 0.8)
					{
						minOccupied = j;
						double dist = (double)measData.range[i]/map.resolution - minOccupied;
						q += ((double)zHit*helper.probGet(dist, sigmaHit) + (double)zRand/MAX_LASER);
						break;
					}
				}
			
			}
			
		}
		return q;
	}
}
