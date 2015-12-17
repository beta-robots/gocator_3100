#include "gocator3100.h"

Gocator3100::Device::Device(const std::string & _ip_address)
{
	kStatus status;
	kIpAddress ipAddress;
	kChar model_name[50];	
	
	//init all GO API objects
	go_api_ = kNULL;
	go_system_ = kNULL;
	go_sensor_ = kNULL;
	go_dataset_ = kNULL;
	go_stamp_ptr_ = kNULL;
	
	// construct Gocator API Library
	if ((status = GoSdk_Construct(&go_api_)) != kOK)
	{
		std::cout << "Device(). Error: GoSdk_Construct: " << status << std::endl;
		status_ = DEVICE_NOT_FOUND;
		return;
	}

	// construct GoSystem object
	if ((status = GoSystem_Construct(&go_system_, kNULL)) != kOK)
	{
		std::cout << "Device(). Error: GoSystem_Construct: " << status << std::endl;
		status_ = DEVICE_NOT_FOUND;
		return;
	}

	// obtain GoSensor object by sensor IP address
	kIpAddress_Parse(&ipAddress, _ip_address.c_str());
	if ((status = GoSystem_FindSensorByIpAddress(go_system_, &ipAddress, &go_sensor_)) != kOK)
	{
		std::cout << "Device(). Error: GoSystem_FindSensorByIpAddress: " << status << std::endl;
		status_ = DEVICE_NOT_FOUND;
		return;
	}
	
	//Success case. Set status and device fixed params (ip, model name and serial number ).
	status_ = DEVICE_FOUND;
	params_.ip_address_ = _ip_address;
	
	// create connection to GoSensor object
	if ((status = GoSensor_Connect(go_sensor_)) != kOK)
	{
		std::cout << "Device(). Error: GoSensor_Connect: " << status << std::endl;
		status_ = DEVICE_NOT_CONNECT; 
		return;
	}
	status_ = DEVICE_CONNECT;
	
	// enable sensor data channel
	if ((status = GoSystem_EnableData(go_system_, kTRUE)) != kOK)
	{
		std::cout << "Device(). Error: GoSensor_EnableData: " << status << std::endl;
		return;
	}
	
	//Obtain camera model
	if ((status = GoSensor_Model(go_sensor_, model_name, 50)) != kOK )
	{
		std::cout << "Device(). Error: GoSensor_Model: " << status << std::endl;
		return; 
	}
	params_.model_name_ = model_name; 
	
	// start Gocator sensor
	if ((status = GoSystem_Start(go_system_)) != kOK)
	{
		std::cout << "Device(). Error: GoSystem_Start: " << status << std::endl;
		return;
	}
	
	//Obtain camera Serial number
	params_.sn_ = (unsigned int)GoSensor_Id(go_sensor_);
	
	//print info
	std::cout << "Found Sensor: " << std::endl; 
	params_.print(); 
	

}

Gocator3100::Device::~Device()
{
	kStatus status;
	
	// stop Gocator sensor
	if ((status = GoSystem_Stop(go_system_)) != kOK)
	{
		std::cout << "~Device(). Error: GoSystem_Stop: " << status << std::endl;
	}

	// destroy handles
	GoDestroy(go_system_);
	GoDestroy(go_api_);

	//bye bye message
	std::cout << "~Device(). Gocator Sensor Stopped and Device Object Destroyed." << std::endl;
}

int Gocator3100::Device::configure(const DeviceParams & _configs)
{
	
}

// int Gocator3100::Device::connect()
// {
// 	
// }

int Gocator3100::Device::startAquisitionThread()
{
	
}

int Gocator3100::Device::stopAquisitionThread()
{
	
}

int Gocator3100::Device::getCurrentSnapshot(pcl::PointCloud<pcl::PointXYZI> & _p_cloud_) const
{
	
}
int Gocator3100::Device::getSingleSnapshot(pcl::PointCloud<pcl::PointXYZI> & _p_cloud) const
{
	GoDataSet dataset = kNULL;
	GoStamp *stamp = kNULL;
	GoDataMsg dataObj;
	GoMeasurementData *measurementData = kNULL;
	
	//Blocking call up to receive data or timeout
	if (GoSystem_ReceiveData(go_system_, &dataset, RECEIVE_TIMEOUT) == kOK)
	{		 	
		std::cout << "Data message received: " << std::endl; 
		//std::cout << "Dataset count: " << GoDataSet_Count(dataset) << std::endl;
		
		// Loop for each data item in the dataset object
		for (unsigned int ii = 0; ii < GoDataSet_Count(dataset); ii++)
		{			
			//get the data item ii
			dataObj = GoDataSet_At(dataset, ii);
			
			//switch according the type of message
			switch(GoDataMsg_Type(dataObj))
			{
				case GO_DATA_MESSAGE_TYPE_STAMP:
                    {
                        GoStampMsg stampMsg = dataObj;
                        std::cout << "Stamp Message batch count: " << GoStampMsg_Count(stampMsg) << std::endl;
                        for (unsigned int jj = 0; jj < GoStampMsg_Count(stampMsg); jj++)
                        {
                            stamp = GoStampMsg_At(stampMsg, jj);
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
                        for (unsigned int kk = 0; kk < GoMeasurementMsg_Count(measurementMsg); kk++)
                        {
                            measurementData = GoMeasurementMsg_At(measurementMsg, kk);
                            std::cout << "Measurement ID: " << GoMeasurementMsg_Id(measurementMsg) << std::endl;
                            std::cout << "Measurement Value: " << measurementData->value << std::endl;
                            std::cout << "Measurement Decision: " << measurementData->decision << std::endl;
                        }
                    }
					break;	
                    
                case GO_DATA_MESSAGE_TYPE_RANGE:
                    {
                        
                    }
                    break;
			}
		}
		
		//destroys received message
		GoDestroy(dataset);
	}
	else //no message after timeout
	{
		printf ("Error: No data received during the waiting period\n");
	}

	//just testing: set point cloud dimensions
	_p_cloud.width = 26; 
}

int Gocator3100::Device::close()
{
	
}

void Gocator3100::Device::printDeviceData() const
{
	
}
