/**
	CMU ROB-16831 Project 4
	Test.cpp
	Purpose: Test the particle filter simulator

	@author Miao Yu
	@version 1.0 11/25/2015
*/

#include "Simulator.h"

int main()
{
	char mapName[] = "../data/map/wean.dat";
	char logName[] = "../data/log/robotdata1.log";
//	char logName[] = "../data/log/robotdata1_kidnapped.log";
//	char logName[] = "../data/log/ascii-robotdata2.log";
//	char logName[] = "../data/log/ascii-robotdata3.log";
//	char logName[] = "../data/log/ascii-robotdata4.log";
//	char logName[] = "../data/log/ascii-robotdata5.log";

	PFL::Simulator pfSimulator(mapName, logName);
}
