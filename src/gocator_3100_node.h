#ifndef gocator_3100_node_H
#define gocator_3100_node_H

//std
#include <iostream>

//this package
#include "lib/src/gocator3100.h"

//ros dependencies
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h> //PCL-ROS interoperability
#include <pcl_conversions/pcl_conversions.h> //conversions from/to PCL/ROS
#include <std_msgs/Empty.h> //odometry (input)
// #include "gocator_3100/PointCloudAsService.h" //custom "snapshot" service
// #include <sensor_msgs/PointCloud2.h> 

//enum run mode
enum RunMode {SNAPSHOT,PUBLISHER};

/** \brief Gocator3100 ROS wrapping class
 * 
 * Gocator3100 ROS wrapping class
 * 
 * Two running modes:
 *    \* Snapshot upon request
 *    \* Continuous point cloud publisher (not yet implemented) 
 * 
 **/
class Gocator3100Node
{
    protected:
            
        //run mode: The node acts as a server, or a continuous pcl publisher
        RunMode run_mode_;
        
        //Indicates if a request has arrived
        bool is_request_; 
            
        //Device object with HW API
        Gocator3100::Device g3100_camera_;
        
        //ros node handle
        ros::NodeHandle nh_;
        
        //Subscriber. Snapshots requests arrive to this topic as an std_msgs::Empty
        ros::Subscriber snapshot_request_;
        
        //Publisher. Snapshots are published through this topic
        ros::Publisher snapshot_publisher_; 
        
        //A pcl point cloud, used to get data and publish it
        pcl::PointCloud<pcl::PointXYZ> cloud_; 

        //point cloud server
        //ros::ServiceServer pcl_server_; 
        
        //node parameters
        ros::Rate rate_; //loop rate
        std::string frame_name_; //name of the frame of references with respect cloud are published
        
        //camera device parameters
        Gocator3100::DeviceConfigs device_params_;
        
    public:
        //constructor
        Gocator3100Node();
        
        //destructor
        ~Gocator3100Node();
        
        //returns run_mode_
        RunMode runMode() const;
        
        //returns is_request_
        bool isRequest() const;
        
        //sets is_request_ to false
        void resetRequest();
        
        //Call to device snapshot acquisition and publish the point cloud
        void publish();
        
        //sleep up to adjusting loop rate
        void sleep(); 
                        
        //Service callback implementing the point cloud snapshot
        //bool pointCloudSnapshotService(gocator_3100::PointCloudAsService::Request  & _request, gocator_3100::PointCloudAsService::Response & _reply);
        
    protected: 
        //snapshot request callback
        void snapshotRequestCallback(const std_msgs::Empty::ConstPtr& _msg);
                
};
#endif
