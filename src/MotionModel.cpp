/**
	CMU ROB-16831 Project 4
	MotionModel.cpp
	Purpose: Implement a motion model

	@author Miao Yu
	@version 1.0 11/26/2015
*/

#include "MotionModel.h"
#include <cmath>

namespace PFL
{
	MotionModel::MotionModel(){}

	MotionModel::~MotionModel(){}

	Particle MotionModel::sampling(Particle state, 
		OdomData odomPrev, OdomData odomCurr)
	{
		double rot1 = atan2(odomCurr.y - odomPrev.y,
			odomCurr.x - odomPrev.x) - odomPrev.theta;
		double trans = sqrt(pow(odomCurr.y - odomPrev.y, 2) + 
			pow(odomCurr.x - odomPrev.x, 2));
		double rot2 = odomCurr.theta - odomPrev.theta -rot1;

		double newRot1 = rot1 - helper.probSample(alpha1 * pow(rot1,2) +
			alpha2 * pow(trans,2), -M_PI, M_PI);
		double newTrans = trans - helper.probSample(alpha3 * pow(trans,2) + 
			alpha4 * pow(rot1,2) + alpha4 * pow(rot2,2));
		double newRot2 = rot2 - helper.probSample(alpha1 * pow(rot2,2) +
			alpha2 * pow(trans,2), -M_PI, M_PI);

		Particle updatedState;
		updatedState.x = state.x + newTrans * cos(state.theta + newRot1);
		updatedState.y = state.y + newTrans * sin(state.theta + newRot1);
		updatedState.theta = state.theta + newRot1 + newRot2;

		return updatedState;
	}

}
