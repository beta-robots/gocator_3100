#ifndef Gocator3100_H
#define Gocator3100_H

//std c/c++
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

//GoSdk
#include <GoSdk/GoSdk.h>

//PCL
#include <pcl/point_types.h>
 #include <pcl/io/pcd_io.h>

//constants
#define SENSOR_IP "192.168.1.10"	
#define RECEIVE_TIMEOUT 20000000

namespace Gocator3100
{

//status  values
enum {DEVICE_NOT_FOUND=0,DEVICE_FOUND,DEVICE_NOT_CONNECT,DEVICE_CONNECT,DEVICE_RUNNING};
	
//device parameters struct
struct DeviceParams
{
	std::string ip_address_;	
	std::string model_name_; 
	unsigned int sn_; 
	
	void print()
	{
		std::cout << "\tIP ad: \t" << ip_address_ << std::endl;
		std::cout << "\tModel: \t" << model_name_ << std::endl;
		std::cout << "\tSN: \t" << sn_ << std::endl;	
	}
};

//device configuration struct
struct DeviceConfigs
{
	double exposure_time_;
};

//Device class
class Device
{	
	protected:
		//current point cloud. Maybe not necessary to be here!
		pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_; //p_cloud_  = new pcl::PointCloud<pcl::PointXYZ>
		
		//driver status
		unsigned int status_;
		
		//device fixed params
		DeviceParams params_;
		
		//device configuration
		DeviceParams configs_;
		
		//GO API objects
		kAssembly go_api_;
		GoSystem go_system_;
		GoSensor go_sensor_;
		GoDataSet go_dataset_;
		GoStamp *go_stamp_ptr_;
		
	public:
		/** \brief Constructor
		 * 
		 * Constructor. Sets params_ struct.
		 * 
		 **/
		Device(const std::string & _ip_address); 

		/** \brief Destructor
		 * 
		 * Destructor
		 * 
		 **/		
		~Device();
		
		/** \brief Connect to a pysical device given an ip address
		 * 
		 * Connect to a pysical device given an ip address
		 * 
		 **/		
// 		int connect();
		
		/** \brief Set/get device parameters to/from the camera
		 * 
		 * Set/get device parameters to/from the camera
		 * 
		 **/
		int configure(const DeviceParams & _configs);
		
		/** \brief Start continuous acquisition in a searated thread
		 * 
		 * Start continuous acquisition in a searated thread
		 * 
		 **/
		int startAquisitionThread(); 
		
		/** \brief Stop the acquisition thread
		 * 
		 * Stop the acquisition thread
		 * 
		 **/
		int stopAquisitionThread();
		
		/** \brief Get the current snapshot 
		 * 
		 * Get the current snapshot, when in continuous acquisition
		 * 
		 **/		
		int getCurrentSnapshot(pcl::PointCloud<pcl::PointXYZI> & _p_cloud_) const; 

		/** \brief Get a single snapshot in the same thread
		 * 
		 * Get a single snapshot in the same thread
		 * 
		 **/				
		int getSingleSnapshot(pcl::PointCloud<pcl::PointXYZI> & _p_cloud) const;
		
		/** \brief Close the connection to a physical device
		 * 
		 * Close the connection to a physical device
		 * 
		 **/				
		int close();
		
		/** \brief Print the device configuration
		 * 
		 * Print the device configuration
		 * 
		 **/				
		void printDeviceData() const; 
};//close class
}//close namespace

#endif
