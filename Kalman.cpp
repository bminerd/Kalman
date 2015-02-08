//============================================================================
// Name        : EigenTest.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <KalmanFilter.h>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::Matrix;

int main()
{
    float dt = 0.02f;

    KalmanFilter<float, 12, 3> kalmanFilter;
    KalmanFilter<float, 6, 3> kalmanFilter6;

    kalmanFilter.getFMatrix() <<
        1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,  -dt, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,  -dt, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,  -dt,
		0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,   dt, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,   dt, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,   dt, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;

    kalmanFilter6.getFMatrix() <<
        1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;

    kalmanFilter.getBMatrix() <<
	      dt, 0.0f, 0.0f,
	    0.0f,   dt, 0.0f,
	    0.0f, 0.0f,   dt,
	    0.0f, 0.0f, 0.0f,
	    0.0f, 0.0f, 0.0f,
	    0.0f, 0.0f, 0.0f,
	    0.0f, 0.0f, 0.0f,
	    0.0f, 0.0f, 0.0f,
	    0.0f, 0.0f, 0.0f,
	    0.0f, 0.0f, 0.0f,
	    0.0f, 0.0f, 0.0f,
	    0.0f, 0.0f, 0.0f;

    kalmanFilter6.getBMatrix() <<
          dt, 0.0f, 0.0f,
        0.0f,   dt, 0.0f,
        0.0f, 0.0f,   dt,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f;

    kalmanFilter.getHMatrix() <<
	    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

    kalmanFilter6.getHMatrix() <<
        1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;

	float p = 1.0f;

	kalmanFilter.getPMatrix() <<
	       p, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f,    p, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f,    p, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,    p, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,    p, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    p, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    p, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    p, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    p, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    p, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    p, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    p;

	kalmanFilter6.getPMatrix() <<
           p, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f,    p, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f,    p, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,    p, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,    p, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    p;

	float q = 0.2f * dt;

	kalmanFilter.getQMatrix() <<
           q, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f,    q, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f,    q, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,    q, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,    q, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    q, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    q, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    q, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    q, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    q, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    q, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    q;

	kalmanFilter6.getQMatrix() <<
           q, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f,    q, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f,    q, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,    q, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,    q, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    q;

	float r = 0.6f;

	kalmanFilter.getRMatrix() <<
           r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f,    r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f,    r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,    r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,    r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    r, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    r, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

	kalmanFilter6.getRMatrix() <<
           r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f,    r, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f,    r, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,    r, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,    r, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f,    r;

	float accelConversionFactor = 0.0039f;
	float gyroConversionFactor  = 1.0f / 14.375f;

	int16_t accelRawData[1000][3];
	float accelData[1000][3];
	int16_t gyroRawData[1000][3];
	float gyroData[1000][3];
	int16_t magRawData[1000][3];
	float magData[1000][3];

	uint16_t dTime;

	ifstream inputFile;
//	inputFile.open("imu_output_pitch_roll_3.csv");
	inputFile.open("imu_output_3.csv");
	string line;

	int i = 0;

	while (getline(inputFile, line) && i < 1000)
	{
	    sscanf(line.data(),
	           "%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\n",
	           &(accelRawData[i][0]),
	           &(accelRawData[i][1]),
	           &(accelRawData[i][2]),
	           &(gyroRawData[i][0]),
	           &(gyroRawData[i][1]),
	           &(gyroRawData[i][2]),
	           &(magRawData[i][0]),
	           &(magRawData[i][1]),
	           &(magRawData[i][2]),
	           &dTime);

	    accelData[i][0] = (float) accelRawData[i][0] * accelConversionFactor;
        accelData[i][1] = (float) accelRawData[i][1] * accelConversionFactor;
        accelData[i][2] = (float) accelRawData[i][2] * accelConversionFactor;

        gyroData[i][0] = (float) gyroRawData[i][0] * gyroConversionFactor;
        gyroData[i][1] = (float) gyroRawData[i][1] * gyroConversionFactor;
        gyroData[i][2] = (float) gyroRawData[i][2] * gyroConversionFactor;

        magData[i][0] = (float) magRawData[i][0];
        magData[i][1] = (float) magRawData[i][1];
        magData[i][2] = (float) magRawData[i][2];

	    i++;
	}

	inputFile.close();


	int nSamples = i + 1;

	int nSamplesToAverage = 50;

	float accelSums[3] = {0.0f, 0.0f, 0.0f};;
	float gyroSums[3] = {0.0f, 0.0f, 0.0f};
	float magSums[3] = {0.0f, 0.0f, 0.0f};

	float accelMeans[3] = {0.0f, 0.0f, 0.0f};
	float gyroMeans[3] = {0.0f, 0.0f, 0.0f};
	float magMeans[3] = {0.0f, 0.0f, 0.0f};

	// Find sum of first N accel, gyro, and mag samples
	for (i = 0; i < nSamplesToAverage; i++)
	{
	    accelSums[0] += accelData[i][0];
	    accelSums[1] += accelData[i][1];
	    accelSums[2] += accelData[i][2];

	    gyroSums[0] += gyroData[i][0];
	    gyroSums[1] += gyroData[i][1];
	    gyroSums[2] += gyroData[i][2];

	    magSums[0] += magData[i][0];
	    magSums[1] += magData[i][1];
	    magSums[2] += magData[i][2];
	}

	// Calculate mean of accel, gyro, and mag sums
	// (this will be used to remove the sensor offset)

	accelMeans[0] = accelSums[0] / (float) nSamplesToAverage;
	accelMeans[1] = accelSums[1] / (float) nSamplesToAverage;
	accelMeans[2] = accelSums[2] / (float) nSamplesToAverage;

    gyroMeans[0] = gyroSums[0] / (float) nSamplesToAverage;
    gyroMeans[1] = gyroSums[1] / (float) nSamplesToAverage;
    gyroMeans[2] = gyroSums[2] / (float) nSamplesToAverage;

    magMeans[0] = magSums[0] / (float) nSamplesToAverage;
    magMeans[1] = magSums[1] / (float) nSamplesToAverage;
    magMeans[2] = magSums[2] / (float) nSamplesToAverage;

    ofstream filteredStatesFile;
    filteredStatesFile.open("filtered_states.csv");

    ofstream filteredStatesFile6;
    filteredStatesFile6.open("filtered_states_6.csv");

    ofstream accelAnglesFile;
    accelAnglesFile.open("accel_angles.csv");

    float lastMeasurement[12] =
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float lastMeasurement6[6] =
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // TEST
//    nSamples = 1;

    for (i = 0; i < nSamples; i++)
	{
        float accelX = accelData[i][0];
	    float accelY = accelData[i][1];
	    float accelZ = accelData[i][2];

	    float accelXMean = accelX - accelMeans[0];
	    float accelYMean = accelY - accelMeans[1];
	    float accelZMean = accelZ - accelMeans[2];

	    float gyroX = gyroData[i][0] - gyroMeans[0];
	    float gyroY = gyroData[i][1] - gyroMeans[1];
	    float gyroZ = gyroData[i][2] - gyroMeans[2];

	    float magX = magData[i][0];
	    float magY = magData[i][1];
	    float magZ = magData[i][2];

	    float pX = 0.0f;
	    float pY = 0.0f;
	    float pZ = 0.0f;

	    float vX = lastMeasurement[3] + (accelXMean * dt);
	    float vY = lastMeasurement[4] + (accelYMean * dt);
	    float vZ = lastMeasurement[5] + (accelZMean * dt);

	    float accelAngleZ = accelZ + (1.0f - accelMeans[2]);

	    // Calculate attitude_x from y and z component of gravity vector
	    float aX = atan2(accelY, accelAngleZ) * (180.0f / 3.14f);
	    // Calculate attitude_y from x and z component of gravity vector
	    float aY = atan2(accelX, accelAngleZ) * (180.0f / 3.14f);
	    // Calculate attitude_z from x and z component of mag reading
	    // (probably will not be accurate)
	    float aZ = atan2(magX, magZ) * (180.0f / 3.14f);

	    accelAnglesFile << aX << ", " << aY << ", " << aZ << endl;

	    float measurement[12] =
	    {
	        aX,
            aY,
            aZ,
	        pX,
	        pY,
	        pZ,
	        vX,
	        vY,
	        vZ,
	        0.0f,
	        0.0f,
	        0.0f
	    };

	    float measurement6[6] =
        {
            aX,
            aY,
            aZ,
            pX,
            pY,
            pZ
        };

	    kalmanFilter.getUMatrix() << gyroX, gyroY, gyroZ;
	    kalmanFilter6.getUMatrix() << gyroX, gyroY, gyroZ;

	    // Predict
	    kalmanFilter.predict();
	    kalmanFilter6.predict();

	    kalmanFilter.update(measurement);
	    kalmanFilter6.update(measurement6);

	    filteredStatesFile <<
	        kalmanFilter.getXMatrix()(0, 0) << ", " <<
	        kalmanFilter.getXMatrix()(1, 0) << ", " <<
	        kalmanFilter.getXMatrix()(2, 0) << ", " <<
	        kalmanFilter.getXMatrix()(3, 0) << ", " <<
	        kalmanFilter.getXMatrix()(4, 0) << ", " <<
	        kalmanFilter.getXMatrix()(5, 0) << ", " <<
	        kalmanFilter.getXMatrix()(6, 0) << ", " <<
	        kalmanFilter.getXMatrix()(7, 0) << ", " <<
	        kalmanFilter.getXMatrix()(8, 0) << ", " <<
	        kalmanFilter.getXMatrix()(9, 0) << ", " <<
	        kalmanFilter.getXMatrix()(10, 0) << ", " <<
	        kalmanFilter.getXMatrix()(11, 0);
	    filteredStatesFile << endl;

	    filteredStatesFile6 <<
            kalmanFilter6.getXMatrix()(0, 0) << ", " <<
            kalmanFilter6.getXMatrix()(1, 0) << ", " <<
            kalmanFilter6.getXMatrix()(2, 0) << ", " <<
            kalmanFilter6.getXMatrix()(3, 0) << ", " <<
            kalmanFilter6.getXMatrix()(4, 0) << ", " <<
            kalmanFilter6.getXMatrix()(5, 0);
	    filteredStatesFile6 << endl;

	    memcpy(measurement, lastMeasurement, 12 * 4);
	    memcpy(measurement6, lastMeasurement6, 6 * 4);
	}

	filteredStatesFile.close();
	filteredStatesFile6.close();
	accelAnglesFile.close();

	return 0;
}
