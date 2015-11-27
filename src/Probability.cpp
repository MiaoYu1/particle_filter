/**
	CMU ROB-16831 Project 4
	Probability.cpp
	Purpose: Implement some helper functions for 
				motion model and measurement model

	@author Miao Yu
	@version 1.0 11/26/2015
*/

#include "Probability.h"
#include <random>
#include <chrono>
#include <cmath>

namespace PFL
{
	Probability::Probability(){}

	Probability::~Probability(){}

	double Probability::probGet(double dist, double sigmaSquare)
	{
		double sigma = sqrt(sigmaSquare);
		double sqrt2pi = sqrt(2*M_PI);
		return 1/sigma/sqrt2pi * exp(-pow(dist,2)/2/sigmaSquare);
	}

	double Probability::probSample(double sigmaSquare)
	{
		double sigma = sqrt(sigmaSquare);
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine generator (seed);
		std::normal_distribution<double> distribution(0,sigma);

		return distribution(generator);
	}

	double Probability::probSample(double sigmaSquare, 
		double lowerBound, double upperBound)
	{
		double sigma = sqrt(sigmaSquare);
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine generator (seed);
		std::normal_distribution<double> distribution(0,sigma);

		while (1)
		{
			double result = distribution(generator);
			if (result >= lowerBound && result <= upperBound)
				return result;
		} 
	}
}