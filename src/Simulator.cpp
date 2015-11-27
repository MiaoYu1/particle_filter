/**
	CMU ROB-16831 Project 4
	Simulator.cpp
	Purpose: A simulator which runs particle filter

	@author Miao Yu
	@version 1.0 11/25/2015
*/
#include "Simulator.h"
#include <cstdio>
#include <iostream>

// #define DEBUG

namespace PFL
{
	using namespace cv;

	Simulator::Simulator(char *inputMapName, char *inputLogName)
	{
		mapName = inputMapName;
		logName = inputLogName;

		namedWindow("Wean", WINDOW_AUTOSIZE);

		if (!readMap())
			drawMap();

		readLog();
		execution();
	}

	Simulator::~Simulator()
	{
		destroyWindow("Wean");
	}

	int Simulator::readMap()
	{
		int x, y, count;
		float temp;
		char line[256];
		FILE *fp;

		if ((fp = fopen(mapName, "rt")) == NULL)
		{
			fprintf(stderr, "# Could not open file %s\n", mapName);
			return -1;
		}
		fprintf(stderr, "# Reading map: %s\n", mapName);
		
		while ((fgets(line, 256, fp) != NULL)
			&& (strncmp("global_map[0]", line , 13) != 0)) 
		{
			if (strncmp(line, "robot_specifications->resolution", 32) == 0)
				if (sscanf(&line[32], "%d", &(map.resolution)) != 0)
					printf("# Map resolution: %d cm\n", map.resolution);

			if (strncmp(line, "robot_specifications->autoshifted_x", 35) == 0)
				if (sscanf(&line[35], "%g", &(map.offsetX)) != 0) 
					printf("# Map offsetX: %g cm\n", map.offsetX);

			if (strncmp(line, "robot_specifications->autoshifted_y", 35) == 0) 
				if (sscanf(&line[35], "%g", &(map.offsetY)) != 0) 
					printf("# Map offsetY: %g cm\n", map.offsetY);
		}

		if (sscanf(line,"global_map[0]: %d %d", &map.sizeY, &map.sizeX) != 2) 
		{
			fprintf(stderr, "ERROR: corrupted file %s\n", mapName);
			fclose(fp);
			return -1;
		}
		printf("# Map size: %d %d\n", map.sizeX, map.sizeY);

		// Allocate the probability array once we get the size of the map
		map.prob.create(map.sizeY, map.sizeX, CV_32FC1);

		map.minX = map.sizeX;
		map.maxX = 0;
		map.minY = map.sizeY;
		map.maxY = 0;
		count = 0;
		for (x = 0; x < map.sizeX; x++)
			for (y = 0; y < map.sizeY; y++, count++) 
			{
				if (count % 10000 == 0)
					fprintf(stderr, "\r# Reading ... (%.2f%%)",
						count / (float)(map.sizeX * map.sizeY) * 100);

				fscanf(fp,"%e", &temp);
				if (temp < 0.0)
					map.prob.at<float>(x,y) = -1;
				else 
				{
					if (x < map.minX)
						map.minX = x;
					else if (x > map.maxX)
						map.maxX = x;
					if (y < map.minY)
						map.minY = y;
					else if (y > map.maxY)
						map.maxY = y;
					map.prob.at<float>(x,y) = temp;	   
				}
			}
		fprintf(stderr, "\r# Reading ... (%.2f%%)\n\n",
			count / (float)(map.sizeX * map.sizeY) * 100);

		fclose(fp);
		return 0;
	}

	int Simulator::readLog()
	{
		char line[1024];
		FILE *fp;

		if ((fp = fopen(logName, "rt")) == NULL)
		{
			fprintf(stderr, "# Could not open file %s\n", logName);
			return -1;
		}
		fprintf(stderr, "# Reading log: %s\n", logName);
		
		sensorLog.size = 0;
		while (fgets(line, 1024, fp) != NULL)
		{
			sensorLog.log.push_back(line);
			sensorLog.size++;
			fprintf(stderr, "\r# Reading ... (%d)", sensorLog.size);
		}
		fprintf(stderr, "\r# Reading ... (%d)\n", sensorLog.size);
		fprintf(stderr, "# log size: %d\n", sensorLog.size);

		return 0;
	}

	void Simulator::execution()
	{
		// Initialize particle filter
		pf = new ParticleFilter(map);
		samples = pf->generateParticles();
		drawParticles();

		OdomData odomPrev, odomCurr;
		int firstOdomFlag = 0;
		int resampleFlag = 0;
		
		for (int i = 0; i < sensorLog.log.size(); i++)
		{
			char *line = new char[sensorLog.log[i].length()+1];
			strcpy(line, sensorLog.log[i].c_str());

			if (strncmp(line, "O", 1) == 0)
			{
				odomCurr = odomDataParse(line);
				if (!firstOdomFlag)
				{
					odomPrev = odomCurr;
					firstOdomFlag = 1;
					drawParticles();
				}
				else if ((pow(odomPrev.x-odomCurr.x,2) + 
					pow(odomPrev.y-odomCurr.y,2) + 
					pow(odomPrev.theta-odomCurr.theta,2)) < 0.01)
					continue;
				else
					resampleFlag = 1;
			
				samples = pf->update(odomPrev, odomCurr);
				odomPrev = odomCurr;
				printf("t = %f\n", odomCurr.ts); // Print current time
			}
			else if (strncmp(line, "L", 1) == 0)
			{
				if (resampleFlag)
				{
					MeasData measurement = measDataParse(line);
					Particle state = pf->update(measurement);
					drawLasers(state, measurement);
					printf("t = %f\n", measurement.ts); // Print current time
					resampleFlag = 0;
				}	
			}
			else
				printf("Invalid data: %s", line);
		}
	}

	OdomData Simulator::odomDataParse(char *line)
	{
		OdomData odometry;

        char delim[] = " ";
        strtok(line, delim); // First character, "O"

        odometry.x = atof(strtok(NULL, delim));
        odometry.y = atof(strtok(NULL, delim));
        odometry.theta = atof(strtok(NULL, delim));
        odometry.ts = atof(strtok(NULL, delim));

        return odometry;
	}

	MeasData Simulator::measDataParse(char *line)
	{
		MeasData measurement;

        char delim[] = " ";
        strtok(line, delim); // First character, "L"

        measurement.x = atof(strtok(NULL, delim));
        measurement.y = atof(strtok(NULL, delim));
        measurement.theta = atof(strtok(NULL, delim));
        measurement.xLaser = atof(strtok(NULL, delim));
        measurement.yLaser = atof(strtok(NULL, delim));
        measurement.thetaLaser = atof(strtok(NULL, delim));
        
        for (int i = 0; i < NUM_RANGE; i++)
        	measurement.range[i] = atoi(strtok(NULL, delim));
        	
        measurement.ts = atof(strtok(NULL, delim));
		
		return measurement;
	}

	void Simulator::drawMap()
	{
		imshow("Wean", map.prob);

#ifdef DEBUG
  		while (waitKey(100) == -1) {}
#endif
	}

	void Simulator::drawParticles()
	{
		Mat background = map.prob;
		cvtColor(background, background, CV_GRAY2BGR);

		for (int i = 0; i < samples.size(); i++)
			circle(background, Point(samples[i].y,samples[i].x), 
				1, Scalar(1,0,0), 2, 8, 0);

		imshow("Wean", background);
		waitKey(1);

#ifdef DEBUG
		printf("Drawing particles\n");
  		while (waitKey(100) == -1) {}
#endif
	}

	void Simulator::drawLasers(Particle state, MeasData measData)
	{
 		Mat background = map.prob;
 		cvtColor(background, background, CV_GRAY2BGR);
		
		// draw particles
		for (int i = 0; i < samples.size(); i++)
			circle(background, Point(samples[i].y,samples[i].x), 
				1, Scalar(1,0,0), 2, 8, 0);
		
		// draw laser		
		double x = state.x;
		double y = state.y;
		double theta = state.theta;
		double resolution = (double)map.resolution;
		
 		for (int i = 0; i < NUM_RANGE; i++)
 		{
 			double xLaser = x + LASER_OFFSET_X * cos(theta) -
				LASER_OFFSET_Y * sin(theta);
			double yLaser = y + LASER_OFFSET_X * sin(theta) +
				LASER_OFFSET_Y * cos(theta);
				
 			double xMeas = x + LASER_OFFSET_X * cos(theta) -
 				LASER_OFFSET_Y * sin(theta) + (double)measData.range[i]/
 				resolution * cos(theta + (double)(i-90) * DEGREE2RADIAN);
			double yMeas = y + LASER_OFFSET_X * sin(theta) +
				LASER_OFFSET_Y * cos(theta) + (double)measData.range[i]/
				resolution * sin(theta + (double)(i-90) * DEGREE2RADIAN);
			
 			line(background, Point(yLaser,xLaser), Point(yMeas,xMeas),
 				Scalar(0,0,1));
 		}
 		
 		imshow("Wean", background);
		waitKey(1);
		
 #ifdef DEBUG
   		while (waitKey(100) == -1) {}
 #endif
	}
}
