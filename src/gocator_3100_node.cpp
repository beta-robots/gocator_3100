#include "gocator_3100_node.h"

Gocator3100Node::Gocator3100Node() :
    run_mode_(SNAPSHOT),
    is_request_(false),
    g3100_camera_("192.168.1.10"), 
    nh_(ros::this_node::getName()),
    rate_(1)
{
    
    //debugging
    run_mode_ = PUBLISHER;
        
    //set the subscriber
    snapshot_request_ = nh_.subscribe("snapshot_request", 1, &Gocator3100Node::snapshotRequestCallback, this);
    
    //set the publisher
    snapshot_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("gocator_snapshot", 1);
    
    //set the server
    //pcl_server_ = nh_.advertiseService("pcl_snapshot", &Gocator3100Node::pointCloudSnapshotService, this);

    //set parameters TODO: To be get from nh_getParam() (yaml file)
    rate_ = 0.1;
    frame_name_ = "gocator";
    device_params_.exposure_time_ = 40000;
    device_params_.spacing_interval_ = 0.1;
    g3100_camera_.configure(device_params_);
}

Gocator3100Node::~Gocator3100Node()
{

}

RunMode Gocator3100Node::runMode() const
{
    return run_mode_;
}

bool Gocator3100Node::isRequest() const
{
    return is_request_;
}

void Gocator3100Node::resetRequest()
{
    is_request_ = false; 
}

void Gocator3100Node::publish()
{    
    //call gocator 
    if ( g3100_camera_.getSingleSnapshot(cloud_) == 1 )
    //if ( g3100_camera_.getSingleSnapshotFake(cloud_) == 1 )
    {
        cloud_.header.stamp = ros::Time::now().toSec(); //TODO: should be set by the Gocator3100::Device class
        cloud_.header.frame_id = frame_name_; //TODO: Should be a node param
        snapshot_publisher_.publish(cloud_);
    }
    else
    {
        std::cout << "Gocator3100Node::publish(): Error with point cloud snapshot acquisition" << std::endl;
    }
}

void Gocator3100Node::sleep()
{
    rate_.sleep();
}

void Gocator3100Node::snapshotRequestCallback(const std_msgs::Empty::ConstPtr& _msg)
{
    is_request_ = true;
}
                
// bool Gocator3100Node::pointCloudSnapshotService(gocator_3100::PointCloudAsService::Request  & _request, gocator_3100::PointCloudAsService::Response & _reply)
// {
//     //create a pcl point cloud
//     pcl::PointCloud<pcl::PointXYZ> cloud; 
//     
//     std::cout << "Processing service request!" << std::endl;
//     
//     //call gocator 
//     //if ( g3100_camera_.getSingleSnapshot(cloud) == 1 )
//     if ( g3100_camera_.getSingleSnapshotFake(cloud) == 1 )
//     {
//         _reply.pcloud.header.frame_id = "gocator_3100";
//         _reply.pcloud.header.stamp = ros::Time::now();
//         pcl::toROSMsg(cloud,_reply.pcloud);
//         return true;
//     }
//     else
//         return false;
// }
