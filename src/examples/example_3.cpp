//just open a sensor, get one snapshot and close

//std c/c++
#include <iostream> //cout
#include <thread>
#include <chrono>

//GoSdk
#include <GoSdk/GoSdk.h>

//constants
#define SENSOR_IP "192.168.1.10"	
#define RECEIVE_TIMEOUT (20000000)

//main
int main(int argc, char **argv)
{
	kStatus status;
	kAssembly api = kNULL;
	GoSystem system = kNULL;
	GoSensor sensor = kNULL;
	kIpAddress ipAddress;
	kChar model_name[50]; 
	GoDataSet dataset = kNULL;
	GoStamp *stamp = kNULL;
	GoProfilePositionX positionX = kNULL;
	GoDataMsg dataObj;
	GoMeasurementData *measurementData = kNULL;
	unsigned int i,j,k;

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
	
	//gets the sensor model
	if ((status = GoSensor_Model(sensor, model_name, 50)) != kOK )
	{
		std::cout << "Error: GoSensor_Model: " << status << std::endl;
		return -1; 
	}

	//prints sensor info
	std::cout << "Connected to Sensor: " << std::endl; 
	std::cout << "\tModel: \t" << model_name << std::endl;
	std::cout << "\tIP: \t" << SENSOR_IP << std::endl;
	std::cout << "\tSN: \t" << GoSensor_Id(sensor) << std::endl;	
	std::cout << "\tState: \t" << GoSensor_State(sensor) << std::endl;
	
	// start Gocator sensor
	if ((status = GoSystem_Start(system)) != kOK)
	{
		std::cout << "Error: GoSystem_Start: " << status << std::endl;
		return -1;
	}

	//Get data or just sleep for a while
	std::cout << "Sensor is running ..." << std::endl; 
	//std::this_thread::sleep_for(std::chrono::seconds(5));
	
	if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK)
	{		 	
		std::cout << "Data message received: " << std::endl; 
		std::cout << "Dataset count: " << GoDataSet_Count(dataset) << std::endl;
		// each result can have multiple data items
		// loop through all items in result message
		for (i = 0; i < GoDataSet_Count(dataset); ++i)
		{			
			dataObj = GoDataSet_At(dataset, i);
			//Retrieve GoStamp message
			switch(GoDataMsg_Type(dataObj))
			{
			case GO_DATA_MESSAGE_TYPE_STAMP:
				{
					GoStampMsg stampMsg = dataObj;

					std::cout << "Stamp Message batch count: " << GoStampMsg_Count(stampMsg) << std::endl;
					for (j = 0; j < GoStampMsg_Count(stampMsg); ++j)
					{
						stamp = GoStampMsg_At(stampMsg, j);
						std::cout << "  Timestamp: " << stamp->timestamp << std::endl;
						std::cout << "  Encoder: " << stamp->encoder << std::endl;
						std::cout << "  Frame index: " << stamp->frameIndex << std::endl;					
					}
				}
				break;
			case GO_DATA_MESSAGE_TYPE_MEASUREMENT:			
				{
					GoMeasurementMsg measurementMsg = dataObj;

					std::cout << "Measurement Message batch count: " << GoMeasurementMsg_Count(measurementMsg) << std::endl;

					for (k = 0; k < GoMeasurementMsg_Count(measurementMsg); ++k)
					{
						measurementData = GoMeasurementMsg_At(measurementMsg, k);
						std::cout << "Measurement ID: " << GoMeasurementMsg_Id(measurementMsg) << std::endl;
						std::cout << "Measurement Value: " << measurementData->value << std::endl;
						std::cout << "Measurement Decision: " << measurementData->decision << std::endl;
					}
				}
				break;			
			}
		}
		GoDestroy(dataset);
	}
	else
	{
		printf ("Error: No data received during the waiting period\n");
	}
	
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
	std::cout << "Sensor Stopped. Program finished !" << std::endl;
	return 1;
}