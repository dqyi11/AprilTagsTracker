#include <ros/ros.h>
#include "apriltags_tracker/april_tags_turtlebot_controller.h"

int main( int argc, char** argv ) {
  ros::init( argc, argv, "apriltags_turtlebot_controller" );
  AprilTagsTurtlebotController controller;  
  ros::spin();
  return 0;
}
