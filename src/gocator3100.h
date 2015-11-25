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
#define RECEIVE_TIMEOUT (20000000)

//Gocator3100 class
class Gocator3100
{	
	protected:
		//current point cloud. Maybe not necessary to be here!
		pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_; //p_cloud_  = new pcl::PointCloud<pcl::PointXYZ>
		
		//driver status
		unsigned int status_; 
		
		//GO API objects
		kAssembly api = kNULL;
		GoSystem system = kNULL;
		GoSensor sensor = kNULL;
		kIpAddress ipAddress;
		kChar model_name[50]; 
		GoDataSet dataset = kNULL;
		GoStamp *stamp = kNULL;
		
	public:
		Gocator3100(); 
		~Gocator3100();
		int configure();
		int connect(const std::string _ip_address);
		int startAquisitionThread(); 
		int stopAquisitionThread();
		int getCurrentSnapshot(pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_) const; 
		int getSingleSnapshot(pcl::PointCloud<pcl::PointXYZI>::Ptr _p_cloud) const;
		int close();
		void printDeviceData() const; 
};
#endif
