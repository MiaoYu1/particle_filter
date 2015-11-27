/**
	CMU ROB-16831 Project 4
	ParticleFilter.cpp
	Purpose: Implement a particle filter

	@author Miao Yu
	@version 1.0 11/26/2015
*/

#include "ParticleFilter.h"
#include "constants.h"
#include <ctime>
#include <cstdlib>
#include <cmath>

namespace PFL
{
	ParticleFilter::ParticleFilter(MapType inputMap)
	{
		map = inputMap;
		measurementModel = new MeasurementModel(map);
	}

	ParticleFilter::~ParticleFilter(){}

	std::vector<Particle> ParticleFilter::generateParticles()
	{
		int sampleCount = 0;
		int mapSize = map.sizeX * map.sizeY;

		std::srand(std::time(NULL));
		while (sampleCount < NUM_PARTICLE)
		{
			int randomIndexPos = std::rand() % mapSize;

			Particle sampleTemp;
			sampleTemp.x = randomIndexPos % map.sizeY;
			sampleTemp.y = randomIndexPos / map.sizeY;

			if (map.prob.at<float>(sampleTemp.x,sampleTemp.y) == 1)
			{
				int randomIndexOrient = std::rand() % NUM_RANGE;
				sampleTemp.theta = randomIndexOrient;

				samples.push_back(sampleTemp);
				sampleCount++;
			}
		}

		return samples;
	}

	std::vector<Particle> ParticleFilter::update(OdomData odomPrev, 
		OdomData odomCurr)
	{
		std::vector<Particle> samplesPrediction;

		odomPrev.x /= (double)map.resolution;
		odomPrev.y /= (double)map.resolution;
		odomCurr.x /= (double)map.resolution;
		odomCurr.y /= (double)map.resolution;
		
		for (int i = 0; i < samples.size(); i++)
		{
			// Map-based motion model
			Particle sampleTemp;
			int loopCount = 0;

			while (loopCount < 5)
			{
				sampleTemp = motionModel->sampling(samples[i], 
					odomPrev, odomCurr);
				if (sampleTemp.x > map.maxX || sampleTemp.x < map.minX ||
					sampleTemp.y > map.maxY || sampleTemp.y < map.minY ||
					map.prob.at<float>(sampleTemp.x,sampleTemp.y) <= 0.8)
					loopCount++;
				else
					break;
			}	
			if (loopCount < 5)	
				samplesPrediction.push_back(sampleTemp);
		}
		
		samples.clear();
		samples = samplesPrediction;

		return samples;
	}

	Particle ParticleFilter::update(MeasData meas)
	{
		double weightSum = 0;
		double weightMax = 0;
		int maxIndex = -1;
		for (int i = 0; i < samples.size(); i++)
		{
			samples[i].w = measurementModel->weightUpdate(samples[i], meas);
			weightSum += samples[i].w;
		}

		for (int i = 0; i < samples.size(); i++)
		{
			samples[i].w /= weightSum;
			if (samples[i].w > weightMax)
			{
				weightMax = samples[i].w;
				maxIndex = i;
			}
		}
		
		resampling();

		return samples[maxIndex];
	}

	void ParticleFilter::resampling()
	{
		std::vector<Particle> samplesUpdate;
		
		// Mean square error of the position of all particles
		double xSum = 0;
		double ySum = 0;
		double xMean, yMean;
		for (int i = 0; i < samples.size(); i++)
		{
			xSum += samples[i].x;
			ySum += samples[i].y;
		}
		xMean = xSum / samples.size();
		yMean = ySum / samples.size();
		
		double xSquareErrSum = 0;
		double ySquareErrSum = 0;
		double xSquareErr, ySquareErr;
		for (int i = 0; i < samples.size(); i++)
		{
			xSquareErrSum += pow(samples[i].x-xMean,2);
			ySquareErrSum += pow(samples[i].y-yMean,2);
		}
		xSquareErr = xSquareErrSum / samples.size();
		ySquareErr = ySquareErrSum / samples.size();
		double squareErr = sqrt(xSquareErrSum+ySquareErrSum);
		
		// Sample based on the MSE wrt the position of all particles
		// Using sqrt to get a rational sample number
		int numSample = (int)(20*sqrt(squareErr)) > 500 ? 
			(int)(20*sqrt(squareErr)) : 500; 

		std::srand(std::time(NULL));
		int randomStartInt = std::rand() % numSample;
		double randStart = (double)randomStartInt/numSample/numSample;

		double cumulateProb = samples[0].w;
		double interval = (double)1/numSample;
		int index = 0;

		for (int m = 0; m < numSample; m++)
		{
			double U = randStart + m * interval;
			while (U > cumulateProb)
			{
				if (++index >= samples.size())
					break;
				cumulateProb += samples[index].w;
			}
			samplesUpdate.push_back(samples[index]);
		}
		samples.clear();
		samples = samplesUpdate;
	}
}
