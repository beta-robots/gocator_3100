//just open a sensor, get one snapshot and close

//Gocator3100 Lib
#include "../gocator3100.h"

//constants
#define SENSOR_IP "192.168.1.10"

//main
int main(int argc, char **argv)
{
	//gocator camera
	Gocator3100::Device camera(SENSOR_IP);
	
	//point cloud
	pcl::PointCloud<pcl::PointXYZI> p_cloud_;
	
	//get a single snapshot in this thread
	camera.getSingleSnapshot(p_cloud_);

	//check point cloud dimensions
	std::cout << "p_cloud_ size is: " << p_cloud_.width << std::endl; 
	
	//bye
	return 1;
}