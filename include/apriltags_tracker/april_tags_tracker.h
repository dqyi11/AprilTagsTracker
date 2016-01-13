#ifndef APRIL_TAGS_TRACKER_H_
#define APRIL_TAGS_TRACKER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>

class AprilTagsTracker {
public:
  AprilTagsTracker();
  virtual ~AprilTagsTracker();

  void imageCallback( const sensor_msgs::ImageConstPtr& msg);

  ros::NodeHandle m_nh;
  image_transport::ImageTransport m_it;
  image_transport::Subscriber m_sub;
};

#endif // APRIL_TAGS_TRACKER_H_
