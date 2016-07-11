//open the Gocator camera, get snapshots each 5 seconds and read temperature and health

//Gocator3100 Lib
#include "../gocator3100.h"

//std 
#include <iostream>

//constants
#define SENSOR_IP "192.168.1.10"

//main
int main(int argc, char **argv)
{
    //temperatures
    double internal_temp, projector_temp, laser_temp; 
    
    //gocator camera
    Gocator3100::Device camera(SENSOR_IP);
    
    //configure camera (exposutre and spacing)
    Gocator3100::CaptureParams capture_params;
    capture_params.exposure_time_ = 31000; //useconds
    capture_params.spacing_interval_ = 0.1; //mm
    camera.configure(capture_params);
    
    //point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud_(new pcl::PointCloud<pcl::PointXYZ>());
    

    while (1)
    {
        //get a single snapshot in this thread
        camera.getSingleSnapshot(*p_cloud_);
        
        //get temperatures: internal, projector and laser
        camera.getTemperature(internal_temp, projector_temp, laser_temp);
        
        //output
        std::cout << internal_temp << " , " <<  projector_temp << " , " << laser_temp << std::endl;
        
        //sleep
        sleep(5); 

    }    
    
    //bye
    return 1;
}