//just open a sensor, get one snapshot and close

#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#define SENSOR_IP			    "192.168.1.10"						

void main(int argc, char **argv)
{
	kAssembly api = kNULL;
	kStatus status;
	unsigned int i, j, k;
	GoSystem system = kNULL;
	GoSensor sensor = kNULL;
// 	GoDataSet dataset = kNULL;
// 	GoStamp *stamp = kNULL;
// 	GoProfilePositionX positionX = kNULL;
// 	GoDataMsg dataObj;
	kIpAddress ipAddress;
// 	GoMeasurementData *measurementData = kNULL;

	// construct Gocator API Library
	if ((status = GoSdk_Construct(&api)) != kOK)
	{
		printf("Error: GoSdk_Construct:%d\n", status);
		return;
	}

	// construct GoSystem object
	if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
	{
		printf("Error: GoSystem_Construct:%d\n", status);
		return;
	}

	// Parse IP address into address data structure
	kIpAddress_Parse(&ipAddress, SENSOR_IP);

	// obtain GoSensor object by sensor IP address
	if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK)
	{
		printf("Error: GoSystem_FindSensorByIpAddress:%d\n", status);
		return;
	}

	// create connection to GoSensor object
	if ((status = GoSensor_Connect(sensor)) != kOK)
	{
		printf("Error: GoSensor_Connect:%d\n", status);
		return;
	}

	// enable sensor data channel
	if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
	{
		printf("Error: GoSensor_EnableData:%d\n", status);
		return;
	}
	
	// start Gocator sensor
	if ((status = GoSystem_Start(system)) != kOK)
	{
		printf("Error: GoSystem_Start:%d\n", status);
		return;
	}

	// stop Gocator sensor
	if ((status = GoSystem_Stop(system)) != kOK)
	{
		printf("Error: GoSystem_Stop:%d\n", status);
		return;
	}

	// destroy handles
	GoDestroy(system);
	GoDestroy(api);

	printf("Press any key to continue...\n");
	getchar();
	return;
}
