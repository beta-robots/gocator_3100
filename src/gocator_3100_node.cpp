#include "gocator_3100_node.h"

Gocator3100Node::Gocator3100Node() :
//     run_mode_(SNAPSHOT),
//     g3100_camera_("192.168.1.10"), 
    nh_(ros::this_node::getName()),
    is_request_(false)
{      
    //set the subscriber
    snapshot_request_ = nh_.subscribe("snapshot_request", 1, &Gocator3100Node::snapshotRequestCallback, this);
    
    //set the publisher
    snapshot_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("gocator_snapshot", 1);
    
    //set the server
    //pcl_server_ = nh_.advertiseService("pcl_snapshot", &Gocator3100Node::pointCloudSnapshotService, this);

    //Read params from the yaml configuration file
    std::string ip_addr;
    double int_param; 
    nh_.getParam("ip_address", ip_addr);
    nh_.getParam("run_mode", int_param); this->run_mode_ = (RunMode)int_param;
    nh_.getParam("rate", this->rate_);
    nh_.getParam("frame_name", this->frame_name_);
    nh_.getParam("exposure", this->device_params_.exposure_time_);
    nh_.getParam("spacing", this->device_params_.spacing_interval_);
    
    //create a device object
    g3100_camera_ = new Gocator3100::Device(ip_addr); 
    
    //configure according yaml params
    g3100_camera_->configure(device_params_);
    
    //print
    std::cout << "ROS node Setings: " << std::endl; 
    std::cout << "\trun mode: \t" << run_mode_ << std::endl;
    std::cout << "\trate [hz]: \t" << rate_  << std::endl;
    std::cout << "\tframe name: \t" << frame_name_ << std::endl;
}

Gocator3100Node::~Gocator3100Node()
{
    delete g3100_camera_; 
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
    ros::Time ts;
    
    //call gocator 
    if ( g3100_camera_->getSingleSnapshot(cloud_) == 1 )
    //if ( g3100_camera_.getSingleSnapshotFake(cloud_) == 1 )
    {
        ts = ros::Time::now();
        cloud_.header.stamp = (pcl::uint64_t)(ts.toSec()*1e6); //TODO: should be set by the Gocator3100::Device class
        cloud_.header.frame_id = frame_name_; 
        snapshot_publisher_.publish(cloud_);
    }
    else
    {
        std::cout << "Gocator3100Node::publish(): Error with point cloud snapshot acquisition" << std::endl;
    }
}

double Gocator3100Node::rate() const
{
    return rate_;
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
