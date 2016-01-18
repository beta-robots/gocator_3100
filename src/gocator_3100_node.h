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
#include "gocator_3100/PointCloudAsService.h" //custom "snapshot" service
// #include <sensor_msgs/PointCloud2.h> 

//enum run mode
enum RunMode {SERVICE,PUBLISHER};

/** \brief Gocator3100 ROS wrapping class
 * 
 * Gocator3100 ROS wrapping class
 * Publishes a singel point cloud as a service
 * 
 **/
class Gocator3100Node
{
    protected:
            
        //run mode: The node acts as a server, or a continuous pcl publisher
        RunMode mode_;
            
        //Device object with HW API
        Gocator3100::Device g3100_camera_;
        
        //ros node handle
        ros::NodeHandle nh_;

        //point cloud server
        ros::ServiceServer pcl_server_; 

        //dynamic reconfigure 
        //btr_point_tracker::tracker_paramsConfig config; 
    
    protected: 
        //service callback
        
        //get point cloud
        
    public:
        //constructor
        Gocator3100Node();
        
        //destructor
        ~Gocator3100Node();
                
        //Service callback implementing the point cloud snapshot
        bool pointCloudSnapshot(gocator_3100::PointCloudAsService::Request  & _request, gocator_3100::PointCloudAsService::Response & _reply);
        
};
#endif
