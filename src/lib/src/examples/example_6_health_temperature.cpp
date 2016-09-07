//open the Gocator camera, get snapshots each 5 seconds and read temperature and health

//Gocator3100 Lib
#include "../gocator3100.h"

//std 
#include <cstdlib>
#include <iostream>

//constants
#define SENSOR_IP "10.8.1.10"
#define TEMP_LIMIT 52000

//main
int main(int argc, char **argv)
{
    //temperatures
    double internal_temp, projector_temp, laser_temp; 
    
    //check user params
    if (argc!=2)
    {
        std::cout << "Bad parameters! EXIT" << std::endl;
        return -1;
    }

    //get user param
    unsigned int num_iters = (unsigned int)atoi(argv[1]);
    
    //gocator camera
    Gocator3100::Device camera(SENSOR_IP);
    
    //configure camera (exposutre and spacing)
    Gocator3100::CaptureParams capture_params;
    capture_params.exposure_time_ = 31000; //useconds
    capture_params.spacing_interval_ = 0.1; //mm
    camera.configure(capture_params);
    
    //point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud_(new pcl::PointCloud<pcl::PointXYZ>());
    
    std::cout << "internal_temp" << " , " <<  "projector_temp" << " , " << "laser_temp" << "," << "iter_number" << std::endl;

    //main loop
    for (unsigned int ii=0; ii<num_iters; ii++)
    {
        //get a single snapshot in this thread
        camera.getSingleSnapshot(*p_cloud_);
        
        //get temperatures: internal, projector and laser
        camera.getTemperature(internal_temp, projector_temp, laser_temp);
        
        //output
        std::cout << internal_temp << " , " <<  projector_temp << " , " << laser_temp << "," << ii << std::endl;

        //safe limit for temp
        if( internal_temp >  TEMP_LIMIT)
        {
            std::cout << "Temperature limit reached" << std::endl;
            break;
        }
        
        //sleep
        sleep(5); 
    }


    
    //bye
    return 1;
}