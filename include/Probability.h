/**
	CMU ROB-16831 Project 4
	Probability.h
	Purpose: Implement some helper functions for 
				motion model and measurement model

	@author Miao Yu
	@version 1.0 11/26/2015
*/

#ifndef __PROBABILITY_H_
#define __PROBABILITY_H_

namespace PFL
{
	class Probability
	{
	public:
		/**
			Constructor of the class Probability

			@param None
			@return None
		*/
		Probability();

		/**
			Destructor of the class Probability

			@param None
			@return None
		*/
		~Probability();

		/**
			Get the probablity of a zero-centered normal distribution

			@param dist The distance to zero
			@param sigmaSquare The variance of the normal distribution
			@return The probability
		*/
		double probGet(double dist, double sigmaSquare);

		/**
			Sample from a zeros-centered normal distribution

			@param sigmaSquare The variance of the normal distribution
			@return The sampling result of given distribution
		*/
		double probSample(double sigmaSquare);

		/**
			Sample from a zeros-centered normal distribution

			@param sigmaSquare The variance of the normal distribution
			@param lowerBound The lower bound of the sampling result
			@param upperBound The upper bound of the sampling result
			@return The sampling result of given distribution
		*/
		double probSample(double sigmaSquare, double lowerBound, 
			double upperBound);

	};
}

#endif