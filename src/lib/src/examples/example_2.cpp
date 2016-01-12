//just open a sensor, get one snapshot and close

//std c/c++
#include <iostream> //cout
#include <stdio.h> //sleep ?
#include <stdlib.h> //sleep ?
#include <memory.h> //sleep?

//GoSdk
#include <GoSdk/GoSdk.h>

//constants
#define SENSOR_IP "192.168.1.10"						

//main
int main(int argc, char **argv)
{
	kStatus status;
	kAssembly api = kNULL;
	GoSystem system = kNULL;
	GoSensor sensor = kNULL;
	kIpAddress ipAddress;
// 	GoDataSet dataset = kNULL;
// 	GoStamp *stamp = kNULL;
// 	GoProfilePositionX positionX = kNULL;
// 	GoDataMsg dataObj;
// 	GoMeasurementData *measurementData = kNULL;

	//Hello message
	std::cout << "Gocator example_2 running" << std::endl; 
	
	// construct Gocator API Library
	if ((status = GoSdk_Construct(&api)) != kOK)
	{
		std::cout << "Error: GoSdk_Construct: " << status << std::endl;
		return -1;
	}

	// construct GoSystem object
	if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
	{
		std::cout << "Error: GoSystem_Construct: " << status << std::endl;
		return -1;
	}

	// obtain GoSensor object by sensor IP address
	kIpAddress_Parse(&ipAddress, SENSOR_IP);
	if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK)
	{
		std::cout << "Error: GoSystem_FindSensorByIpAddress: " << status << std::endl;
		return -1;
	}

	// create connection to GoSensor object
	if ((status = GoSensor_Connect(sensor)) != kOK)
	{
		std::cout << "Error: GoSensor_Connect: " << status << std::endl;
		return -1;
	}

	// enable sensor data channel
	if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
	{
		std::cout << "Error: GoSensor_EnableData: " << status << std::endl;
		return -1;
	}

	// start Gocator sensor
	if ((status = GoSystem_Start(system)) != kOK)
	{
		std::cout << "Error: GoSystem_Start: " << status << std::endl;
		return -1;
	}

	//sleep for a while
	sleep(5);
	
	// stop Gocator sensor
	if ((status = GoSystem_Stop(system)) != kOK)
	{
		std::cout << "Error: GoSystem_Stop: " << status << std::endl;
		return -1;
	}

	// destroy handles
	GoDestroy(system);
	GoDestroy(api);

	//bye bye message
	std::cout << "Program finished !" << status << std::endl;
	return 1;
}