#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

// #define SENSOR_IP "192.168.1.10"
#define SENSOR_IP "10.8.1.10"                     

void main(int argc, char **argv)
{
	kAssembly api = kNULL;
	kStatus status;
	GoSystem system = kNULL;
	GoSensor sensor = kNULL;
	kIpAddress ipAddress;

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
		printf("Error: GoSystem_FindSensor:%d\n", status);
		return;
	}

	// create connection to GoSensor object
	if ((status = GoSensor_Connect(sensor)) != kOK)
	{
		printf("Error: GoSensor_Connect:%d\n", status);
		return;
	}

	printf("Restore to factory the sensor\n");
	if ((status = GoSensor_RestoreDefaults(sensor, kFALSE)) != kOK)
	{
		printf("Error: GoSensor_RestoreDefaults:%d\n", status);
		return;
	}
	printf("Done\n");

	// destroy handles
	GoDestroy(system);
	GoDestroy(api);

	printf("Press any key to continue...\n");
	getchar();
	return;
}