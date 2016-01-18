
//ros dependencies
#include "gocator_3100_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "gocator_3100_node");
      
      //create ros wrapper object
      Gocator3100Node gocator;
      
      //set node loop rate
      ros::Rate loop_rate(5);
      
      //node loop 
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce(); 
            
            //relax to fit output rate
            loop_rate.sleep();            
      }
            
      //exit program
      return 0;
}