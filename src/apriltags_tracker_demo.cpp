#include <ros/ros.h>
#include "apriltags_tracker/april_tags_tracker.h"

int main( int argc, char** argv ) {
  ros::init( argc, argv, "apriltags_tracker" );
  for( unsigned int i=0; i < argc; i++ ) {
    std::cout << i << " " << argv[i] << std::endl;
  }
  AprilTagsTracker tracker;  
  if( argc > 1 ) {
    tracker.loadFile( argv[1] );
  }
  ros::spin();
  return 0;
}
