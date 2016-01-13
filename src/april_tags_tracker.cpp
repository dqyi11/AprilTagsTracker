#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "apriltags_tracker/april_tags_tracker.h"

#define APRIL_TAGS_TRACKER_VIEW "April Tags Tracker"


void AprilTagsTracker::imageCallback( const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy( msg, "mono8" ); 
  }
  catch( cv_bridge::Exception& e ) {
    ROS_ERROR( "cv_bridge exce[topm: %s", e.what() );
    return;
  }
  cv::imshow(APRIL_TAGS_TRACKER_VIEW, cv_ptr->image );
  cv::waitKey(30);
}

AprilTagsTracker::AprilTagsTracker() : m_it( m_nh )  {
  cv::namedWindow(APRIL_TAGS_TRACKER_VIEW);
  cv::startWindowThread();
  
  m_sub = m_it.subscribe("/usb_cam/image_raw", 1, &AprilTagsTracker::imageCallback, this);
}

AprilTagsTracker::~AprilTagsTracker() {
  cv::destroyWindow( APRIL_TAGS_TRACKER_VIEW );
}
