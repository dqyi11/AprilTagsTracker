#include <iostream>
#include <fstream>
#include <sstream>
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

#define PI 3.1415926
#define REACH_THRESHOLD 20

float convRadius(float radius) {
  if( radius < 0 ) {
    radius = 2*PI + radius;
  }
  return radius;
}

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
    msg.orientation = convRadius( tag.getXYOrientation() );
    m_pos_pub.publish(msg); 

    if( m_target_poses.size() > 0 && m_target_pos_idx >= 0 ) {
      pair< Pos2D, bool> current_target = m_target_poses[m_target_pos_idx];
      line( cv_ptr->image, Point( tag.cxy.first, tag.cxy.second ), Point( current_target.first.x, current_target.first.y ), Scalar(255,255,0), 2 );

      if( is_current_target_reached( tag.cxy.first, tag.cxy.second, current_target.first.x, current_target.first.y ) ) {
        m_target_poses[m_target_pos_idx].second = false;
        if( m_target_pos_idx < m_target_poses.size()-1 ) {
          m_target_pos_idx ++;
          Pos2D new_target_pos = m_target_poses[m_target_pos_idx].first;
          apriltags_tracker::target_pos msg;
          msg.id = 0;
          msg.t_x = new_target_pos.x;
          msg.t_y = new_target_pos.y;
          msg.t_orientation = 0.0;
          m_t_pos_pub.publish(msg); 
          
        }
      }
    }
  }
  for( unsigned int i=0; i<m_target_poses.size(); i++ ) {
    pair< Pos2D, bool > target = m_target_poses[i];
    if( true == target.second ) {  
      circle( cv_ptr->image, Point2f( target.first.x, target.first.y ), 6, Scalar(255,0,0), 4 ); 
    } 
    else {
      circle( cv_ptr->image, Point2f( target.first.x, target.first.y ), 6, Scalar(255,0,0, 0.4), 4 ); 
    }
  }
  cv::imshow(APRIL_TAGS_TRACKER_VIEW, cv_ptr->image );
  /*
  int key_value = cv::waitKey(30);
  if( key_value == (int)('q') ) {
    ros::shutdown();
  }*/
  if( false == ros::ok() ) {
    ros::shutdown();
  }
}

AprilTagsTracker::AprilTagsTracker( AprilTags::TagCodes codes  ) : m_it( m_nh ) , m_tag_codes( codes )  {
  m_target_pos_idx = -1;
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

  AprilTagsTracker* p_april_tags_tracker = static_cast<AprilTagsTracker*>( param );
  if( EVENT_LBUTTONDOWN == event ) {
   
    if( p_april_tags_tracker ) {
      Pos2D pos;
      pos.x = x;
      pos.y = y;
      p_april_tags_tracker->m_target_poses.push_back( make_pair( pos, true ) );
    }
  }
  else if( EVENT_RBUTTONDOWN == event ) {
    if( p_april_tags_tracker ) {
      p_april_tags_tracker->m_target_pos_idx = 0;
      pair< Pos2D, bool> first_pos = p_april_tags_tracker->m_target_poses[0];
      Pos2D pos = first_pos.first;
      apriltags_tracker::target_pos msg;
      msg.id = 0;
      msg.t_x = pos.x;
      msg.t_y = pos.y;
      msg.t_orientation = 0.0;
      p_april_tags_tracker->m_t_pos_pub.publish(msg); 
    }
  } 
  else if( EVENT_MBUTTONDOWN == event ) {
    if( p_april_tags_tracker ) {
      p_april_tags_tracker->m_target_poses.clear();
    }
  }
}

bool AprilTagsTracker::is_current_target_reached( int x, int y, int target_x, int target_y ) {
  double distance = 0.0;
  distance = sqrt( (x-target_x)*(x-target_x) + (y-target_y)*(y-target_y) );
  if( distance < REACH_THRESHOLD ) {
    return true;
  }
  return false;
}  
 
void AprilTagsTracker::loadFile( char* filename ) {
  ifstream read_file;
  m_target_poses.clear();
  m_target_pos_idx = -1;
  read_file.open( filename );
  if( !read_file.good() ) {
    cout << "FAILED IN OPEN " << filename << endl;
    return;
  }
  std::string line_str;
  while( !read_file.eof() ) {
    getline( read_file , line_str );
    std::cout << line_str << std::endl;
    std::istringstream iss( line_str );
    int x, y;
    if( !( iss >> x >> y ) ){
      break; 
    }
    Pos2D pos;
    pos.x = x;
    pos.y = y;
    std::cout << "READ " << x << " " << y << std::endl;
    m_target_poses.push_back( make_pair( pos, true ) ); 
  }
}
