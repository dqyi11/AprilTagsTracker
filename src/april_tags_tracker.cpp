#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "apriltags_tracker/april_tags_tracker.h"
#include "apriltags_tracker/april_tag_pos.h"
#include "apriltags_tracker/target_pos.h"

using namespace std;
using namespace cv;

#define APRIL_TAGS_TRACKER_VIEW "April Tags Tracker"
#define APRIL_TAG_POS_MSG_NAME  "april_tag_pos"
#define TARGET_POS_MSG_NAME     "target_pos"

void AprilTagsTracker::imageCallback( const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy( msg, "bgr8" ); 
  }
  catch( cv_bridge::Exception& e ) {
    ROS_ERROR( "cv_bridge exce[topm: %s", e.what() );
    return;
  }
  vector<AprilTags::TagDetection> tags = extractTags( cv_ptr->image ); 
  //cout << "NUM of TAGS " << tags.size() << endl;
  for( unsigned int i=0; i<tags.size(); i++ ){
    AprilTags::TagDetection tag = tags[i];
    //cout << tag.id << " " << tag.getXYOrientation() << endl; 
    circle( cv_ptr->image, Point2f( tag.cxy.first, tag.cxy.second ), 2, Scalar(0,255,0), 4 ); 
    line( cv_ptr->image, Point( tag.p[0].first, tag.p[0].second ), Point( tag.p[1].first, tag.p[1].second ), Scalar(0,255,0), 2 );
    line( cv_ptr->image, Point( tag.p[1].first, tag.p[1].second ), Point( tag.p[2].first, tag.p[2].second ), Scalar(0,255,0), 2 );
    line( cv_ptr->image, Point( tag.p[2].first, tag.p[2].second ), Point( tag.p[3].first, tag.p[3].second ), Scalar(0,255,0), 2 );
    line( cv_ptr->image, Point( tag.p[3].first, tag.p[3].second ), Point( tag.p[0].first, tag.p[0].second ), Scalar(0,255,0), 2 );
    double orientation_length = sqrt( pow(tag.p[0].first-tag.p[1].first,2) + pow(tag.p[0].second-tag.p[1].second,2) );
    line( cv_ptr->image, Point( tag.cxy.first, tag.cxy.second ), Point( tag.cxy.first+orientation_length*cos(tag.getXYOrientation()), tag.cxy.second+orientation_length*sin(tag.getXYOrientation()) ), Scalar(0,255,0), 2 );

    apriltags_tracker::april_tag_pos msg;
    msg.id = tag.id;
    msg.x = tag.cxy.first;
    msg.y = tag.cxy.second;
    msg.orientation = tag.getXYOrientation();
    m_pos_pub.publish(msg); 
  
    if( m_target_x > 0 && m_target_y > 0 ) {
      line( cv_ptr->image, Point( tag.cxy.first, tag.cxy.second ), Point( m_target_x, m_target_y ), Scalar(255,255,0), 2 );
    }
  }
  if( m_target_x > 0 && m_target_y > 0 ) {
    circle( cv_ptr->image, Point2f( m_target_x, m_target_y ), 6, Scalar(255,0,0), 4 ); 
  }
  cv::imshow(APRIL_TAGS_TRACKER_VIEW, cv_ptr->image );
  int key_value = cv::waitKey(30);
  if( key_value == (int)('q') ) {
    ros::shutdown();
  }
}

AprilTagsTracker::AprilTagsTracker( AprilTags::TagCodes codes  ) : m_it( m_nh ) , m_tag_codes( codes )  {
  m_target_x = -1;
  m_target_y = -1;
  cv::namedWindow(APRIL_TAGS_TRACKER_VIEW);
  cv::setMouseCallback(APRIL_TAGS_TRACKER_VIEW, mouseClick, this );
  cv::startWindowThread();
  
  m_sub = m_it.subscribe("/usb_cam/image_raw", 1, &AprilTagsTracker::imageCallback, this);
  mp_tag_detector = new AprilTags::TagDetector( m_tag_codes ); 

  m_pos_pub = m_nh.advertise<apriltags_tracker::april_tag_pos>( APRIL_TAG_POS_MSG_NAME, 1000 );
  m_t_pos_pub = m_nh.advertise<apriltags_tracker::target_pos>( TARGET_POS_MSG_NAME, 1000 );
}

AprilTagsTracker::~AprilTagsTracker() {
  if( mp_tag_detector ) {
    delete mp_tag_detector;
    mp_tag_detector = NULL;
  }
  cv::destroyWindow( APRIL_TAGS_TRACKER_VIEW );
}
  
std::vector<AprilTags::TagDetection> AprilTagsTracker::extractTags( cv::Mat& image) {
   cv::Mat gray_img;
   cvtColor( image, gray_img, CV_BGR2GRAY );
   return mp_tag_detector->extractTags( gray_img );
}
  
void AprilTagsTracker::mouseClick(int event, int x, int y, int flags, void* param) {

  if( EVENT_LBUTTONDOWN == event ) {
   
    AprilTagsTracker* p_april_tags_tracker = static_cast<AprilTagsTracker*>( param );
    if( p_april_tags_tracker ) {
      p_april_tags_tracker->m_target_x = x;
      p_april_tags_tracker->m_target_y = y;

      apriltags_tracker::target_pos msg;
      msg.id = 0;
      msg.t_x = x;
      msg.t_y = y;
      msg.t_orientation = 0.0;
      p_april_tags_tracker->m_t_pos_pub.publish(msg); 
    }
  }
}
