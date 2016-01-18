#include "gocator_3100_node.h"

Gocator3100Node::Gocator3100Node() : 
    g3100_camera_("192.168.1.10"), 
    nh_(ros::this_node::getName())
{

    //set the server
    pcl_server_ = nh_.advertiseService("pcl_snapshot", &Gocator3100Node::pointCloudSnapshot, this);
}

Gocator3100Node::~Gocator3100Node()
{

}
                
bool Gocator3100Node::pointCloudSnapshot(gocator_3100::PointCloudAsService::Request  & _request, gocator_3100::PointCloudAsService::Response & _reply)
{
    //create a pcl point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud; 
    
    std::cout << "Processing service request!" << std::endl;
    
    //call gocator 
    //if ( g3100_camera_.getSingleSnapshot(cloud) == 1 )
    if ( g3100_camera_.getSingleSnapshotFake(cloud) == 1 )
    {
        _reply.pcloud.header.frame_id = "gocator_3100";
        _reply.pcloud.header.stamp = ros::Time::now();
        pcl::toROSMsg(cloud,_reply.pcloud);
        return true;
    }
    else
        return false;
}
