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
#define SENSOR_IP "192.168.1.10" // sensor IP address
#define RECEIVE_TIMEOUT 20000000 // timeout for snapshot acquisition
#define INVALID_RANGE_16BIT ((signed short)0x8000) // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data. 
#define DOUBLE_MAX ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.  
#define INVALID_RANGE_DOUBLE ((k64f)-DOUBLE_MAX)   // floating point value to represent invalid range data.    
#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

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
	
	void print() const
	{
		std::cout << "\tIP ad: \t" << ip_address_ << std::endl;
		std::cout << "\tModel: \t" << model_name_ << std::endl;
		std::cout << "\tSN: \t" << sn_ << std::endl;	
	}
};

//device configuration struct
struct CaptureParams
{
	double exposure_time_; //in useconds
    double spacing_interval_; //in millimeters
    
    void print() const
    {
        std::cout << "\texposure [us]: \t" << exposure_time_ << std::endl;
        std::cout << "\tspacing [mm]: \t" << spacing_interval_ << std::endl;
    }
};

//Device class
class Device
{	
	protected:
		//current point cloud. Maybe not necessary to be here!
		//pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_; //p_cloud_  = new pcl::PointCloud<pcl::PointXYZ>
		
		//driver status
		unsigned int status_;
		
		//device fixed params
		DeviceParams device_params_;
		
		//device configuration
		CaptureParams capture_params_;
		
		//GO API objects
		kAssembly go_api_;
		GoSystem go_system_;
		GoSensor go_sensor_;
        GoSetup go_setup_;
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
		int configure(const CaptureParams & _configs);
        
        /** \brief Start device data acquisition
         * 
         * Start device data acquisition
         * 
         **/
        int start(); 
        
        /** \brief Stop device dat acquisition
         * 
         * Stop device data acquisition
         * 
         **/
        int stop();
        
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
		int getCurrentSnapshot(pcl::PointCloud<pcl::PointXYZ> & _p_cloud) const; 

		/** \brief Get a single snapshot in the same thread
		 * 
		 * Get a single snapshot in the same thread
		 * 
		 **/				
		int getSingleSnapshot(pcl::PointCloud<pcl::PointXYZ> & _p_cloud);
        
        /** \brief Get a single FAKE snapshot
         *
         * JUST FOR DEBUGGING WHEN NO CAMERA AVAILABLE
         * 
         **/
        int getSingleSnapshotFake(pcl::PointCloud<pcl::PointXYZ> & _p_cloud);
        
        /** \brief Returns all health data as a string
         *
         * Returns all health data as a string
         * 
         **/
        void getDeviceHealth(std::string & _health_str) const;

        /** \brief Returns three device temperatures
         *
         * Returns three device temperatures: internal, projector and laser
         * 
         **/
        void getTemperature(double & _internal_temp, double & _projector_temp, double & _laser_temp) const;        
        
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
