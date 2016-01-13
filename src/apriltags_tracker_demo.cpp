#include <ros/ros.h>
#include "apriltags_tracker/april_tags_tracker.h"

int main( int argc, char** argv ) {
  ros::init( argc, argv, "apriltags_tracker" );
  AprilTagsTracker tracker;  
  ros::spin();
  return 0;
}
