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
		
	// start Gocator sensor
	if ((status = GoSystem_Start(go_system_)) != kOK)
	{
		std::cout << "Device(). Error: GoSystem_Start: " << status << std::endl;
		return;
    }
    
    //Obtain camera model
    if ((status = GoSensor_Model(go_sensor_, model_name, 50)) != kOK )
    {
        std::cout << "Device(). Error: GoSensor_Model: " << status << std::endl;
        return; 
    }
    params_.model_name_ = model_name; 

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

int Gocator3100::Device::getCurrentSnapshot(pcl::PointCloud<pcl::PointXYZI> & _p_cloud) const
{
	
}
int Gocator3100::Device::getSingleSnapshot(pcl::PointCloud<pcl::PointXYZI> & _p_cloud) const
{
	GoDataSet dataset = kNULL;
	GoStamp *stamp = kNULL;
	GoDataMsg dataObj;
	GoMeasurementData *measurementData = kNULL;
	
	//Blocking call up to receive data or timeout
	if (GoSystem_ReceiveData(go_system_, &dataset, RECEIVE_TIMEOUT) != kOK)
    {
        //no message after timeout
        std::cout << "Error: No data received during the waiting period" << std::endl;
    }
    else
    {
		std::cout << "Data message received: " << std::endl; 
		std::cout << "Dataset count: " << GoDataSet_Count(dataset) << std::endl;
		
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
                    std::cout << "Stamp Message: " << GoStampMsg_Count(stampMsg) << std::endl;
                    for (unsigned int jj = 0; jj < GoStampMsg_Count(stampMsg); jj++)
                    {
                        stamp = GoStampMsg_At(stampMsg, jj);
                        std::cout << "\tTimestamp: " << stamp->timestamp << std::endl;
                        std::cout << "\tEncoder: " << stamp->encoder << std::endl;
                        std::cout << "\tFrame index: " << stamp->frameIndex << std::endl;					
                    }
                }
                break;
                    
                case GO_DATA_MESSAGE_TYPE_SURFACE:            
                {  
                    
                    //cast to GoSurfaceMsg
                    GoSurfaceMsg surfaceMsg = dataObj;
                    
                    //Get general data of the surface
                    unsigned int row_count = GoSurfaceMsg_Length(surfaceMsg); 
                    unsigned int exposure = GoSurfaceMsg_Exposure(surfaceMsg);
                    unsigned int width = GoSurfaceMsg_Width(surfaceMsg);
                    std::cout << "Surface Message" << std::endl; 
                    std::cout << "\tExposure: " << exposure << std::endl; 
                    std::cout << "\tLength: " <<  row_count << std::endl; 
                    std::cout << "\tWidth: " << width << std::endl; 
                    
                    //get offsets and resolutions
                    double xResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
                    double yResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
                    double zResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
                    double xOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
                    double yOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
                    double zOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));
                    
                    //resize the point cloud
                    _p_cloud.resize(row_count*width);

                    //run over all rows
                    for (unsigned int ii = 0; ii < row_count; ii++)
                    {
                        //get the pointer to row 
                        short *data = GoSurfaceMsg_RowAt(surfaceMsg,ii);
                        
                        //run over the width of row ii
                        for (unsigned int jj = 0; jj < width; jj++)
                        {
                            //set xy
                            _p_cloud.points[ii*row_count+jj].x = xOffset + xResolution*jj;
                            _p_cloud.points[ii*row_count+jj].y = yOffset + yResolution*ii;
                            
                            //set z
                            if (data[jj] != INVALID_RANGE_16BIT )
                                _p_cloud.points[ii*row_count+jj].z = zOffset + zResolution*data[jj];
                            else
                                _p_cloud.points[ii*row_count+jj].z = INVALID_RANGE_DOUBLE;
                        }
                    }
                }
                break;
            }
        }
                		
		//destroys received message
		GoDestroy(dataset);
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
