#include "gocator_3100_node.h"

Gocator3100Node::Gocator3100Node() :
//     run_mode_(SNAPSHOT),
//     g3100_camera_("192.168.1.10"), 
    nh_(ros::this_node::getName()),
    is_request_(false)
{      
    //set the subscriber
    snapshot_request_ = nh_.subscribe("snapshot_request", 1, &Gocator3100Node::snapshotRequestCallback, this);
    
    //set the point cloud publisher
    snapshot_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("gocator_snapshot", 1);
    
    //Set the fov marker publisher
    fov_publisher_ = nh_.advertise<visualization_msgs::Marker>("fov_markers",1);
    
    //set the server
    //pcl_server_ = nh_.advertiseService("pcl_snapshot", &Gocator3100Node::pointCloudSnapshotService, this);

    //Read params from the yaml configuration file
    std::string ip_addr;
    int int_param;
    nh_.getParam("ip_address", ip_addr);
    nh_.getParam("run_mode", int_param); this->run_mode_ = (RunMode)int_param;
    nh_.getParam("rate", this->rate_);
    nh_.getParam("frame_name", this->frame_name_);
    nh_.getParam("exposure", this->capture_params_.exposure_time_);
    nh_.getParam("spacing", this->capture_params_.spacing_interval_);
    fov_viz_ = true; //TODO: get it from param server
    
    //create a device object
    g3100_camera_ = new Gocator3100::Device(ip_addr); 
    
    //configure according yaml params
    g3100_camera_->configure(capture_params_);
    
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
    
    //Get snapshot from camera and publish the point cloud
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

void Gocator3100Node::publish_fov()
{
    //fill and publish the field of view wires
    fov_marker_msg_.header.stamp = ros::Time::now(); 
    fov_marker_msg_.header.frame_id = frame_name_;
    fov_marker_msg_.ns = ""; 
    fov_marker_msg_.id = 1;
    fov_marker_msg_.type = visualization_msgs::Marker::LINE_LIST; 
    fov_marker_msg_.action = visualization_msgs::Marker::ADD;
    fov_marker_msg_.pose.position.x = 0; 
    fov_marker_msg_.pose.position.y = 0; 
    fov_marker_msg_.pose.position.z = 0; 
    fov_marker_msg_.pose.orientation.x = 0; 
    fov_marker_msg_.pose.orientation.y = 0; 
    fov_marker_msg_.pose.orientation.z = 0; 
    fov_marker_msg_.pose.orientation.w = 1; 
    fov_marker_msg_.scale.x = 0.001;
    fov_marker_msg_.color.a = 0.9;
    fov_marker_msg_.color.r = 0.6;
    fov_marker_msg_.color.g = 0.2;
    fov_marker_msg_.color.b = 0.2;
    fov_marker_msg_.lifetime = ros::Duration(0);
    fov_marker_msg_.points.resize(24);

    //line 1
    fov_marker_msg_.points[0].x = 0.0335;
    fov_marker_msg_.points[0].y = 0.043;
    fov_marker_msg_.points[0].z = 0.035;
    fov_marker_msg_.points[1].x = 0.0465;
    fov_marker_msg_.points[1].y = 0.044;
    fov_marker_msg_.points[1].z = -0.035;

    //line 2
    fov_marker_msg_.points[2].x = 0.0335;
    fov_marker_msg_.points[2].y = -0.043;
    fov_marker_msg_.points[2].z = 0.035;
    fov_marker_msg_.points[3].x = 0.0465;
    fov_marker_msg_.points[3].y = -0.044;
    fov_marker_msg_.points[3].z = -0.035;

    //line 3
    fov_marker_msg_.points[4].x = -0.0335;
    fov_marker_msg_.points[4].y = -0.043;
    fov_marker_msg_.points[4].z = 0.035;
    fov_marker_msg_.points[5].x = -0.0465;
    fov_marker_msg_.points[5].y = -0.044;
    fov_marker_msg_.points[5].z = -0.035;

    //line 4
    fov_marker_msg_.points[6].x = -0.0335;
    fov_marker_msg_.points[6].y = 0.043;
    fov_marker_msg_.points[6].z = 0.035;
    fov_marker_msg_.points[7].x = -0.0465;
    fov_marker_msg_.points[7].y = 0.044;
    fov_marker_msg_.points[7].z = -0.035;
    
    //line 5
    fov_marker_msg_.points[8].x = -0.0465;
    fov_marker_msg_.points[8].y = 0.044;
    fov_marker_msg_.points[8].z = -0.035;
    fov_marker_msg_.points[9].x = 0.0465;
    fov_marker_msg_.points[9].y = 0.044;
    fov_marker_msg_.points[9].z = -0.035;
    
    //line 6
    fov_marker_msg_.points[10].x = 0.0465;
    fov_marker_msg_.points[10].y = 0.044;
    fov_marker_msg_.points[10].z = -0.035;
    fov_marker_msg_.points[11].x = 0.0465;
    fov_marker_msg_.points[11].y = -0.044;
    fov_marker_msg_.points[11].z = -0.035;

    //line 7
    fov_marker_msg_.points[12].x = 0.0465;
    fov_marker_msg_.points[12].y = -0.044;
    fov_marker_msg_.points[12].z = -0.035;
    fov_marker_msg_.points[13].x = -0.0465;
    fov_marker_msg_.points[13].y = -0.044;
    fov_marker_msg_.points[13].z = -0.035;
    
    //line 8
    fov_marker_msg_.points[14].x = -0.0465;
    fov_marker_msg_.points[14].y = -0.044;
    fov_marker_msg_.points[14].z = -0.035;
    fov_marker_msg_.points[15].x = -0.0465;
    fov_marker_msg_.points[15].y = 0.044;
    fov_marker_msg_.points[15].z = -0.035;
    
    //line 9
    fov_marker_msg_.points[16].x = 0.0335;
    fov_marker_msg_.points[16].y = 0.043;
    fov_marker_msg_.points[16].z = 0.035;
    fov_marker_msg_.points[17].x = 0.0335;
    fov_marker_msg_.points[17].y = -0.043;
    fov_marker_msg_.points[17].z = 0.035;
    
    //line 10
    fov_marker_msg_.points[18].x = 0.0335;
    fov_marker_msg_.points[18].y = -0.043;
    fov_marker_msg_.points[18].z = 0.035;
    fov_marker_msg_.points[19].x = -0.0335;
    fov_marker_msg_.points[19].y = -0.043;
    fov_marker_msg_.points[19].z = 0.035;

    //line 11
    fov_marker_msg_.points[20].x = -0.0335;
    fov_marker_msg_.points[20].y = -0.043;
    fov_marker_msg_.points[20].z = 0.035;
    fov_marker_msg_.points[21].x = -0.0335;
    fov_marker_msg_.points[21].y = 0.043;
    fov_marker_msg_.points[21].z = 0.035;

    //line 12
    fov_marker_msg_.points[22].x = -0.0335;
    fov_marker_msg_.points[22].y = 0.043;
    fov_marker_msg_.points[22].z = 0.035;
    fov_marker_msg_.points[23].x = 0.0335;
    fov_marker_msg_.points[23].y = 0.043;
    fov_marker_msg_.points[23].z = 0.035;
    
    //publish
    fov_publisher_.publish(fov_marker_msg_);
}

double Gocator3100Node::rate() const
{
    return rate_;
}

bool Gocator3100Node::isFovViz() const
{
    return fov_viz_;
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
