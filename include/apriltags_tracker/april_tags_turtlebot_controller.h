#ifndef APRIL_TAGS_TURTLEBOT_CONTROLLER_H_
#define APRIL_TAGS_TURTLEBOT_CONTROLLER_H_

#include <ros/ros.h>
#include "apriltags_tracker/april_tag_pos.h"
#include "apriltags_tracker/target_pos.h"

class AprilTagsTurtlebotController {
public:
  AprilTagsTurtlebotController();
  virtual ~AprilTagsTurtlebotController();

  void positionCallback(const apriltags_tracker::april_tag_pos::ConstPtr& msg);
  void targetPositionCallback(const apriltags_tracker::target_pos::ConstPtr& msg);

  ros::NodeHandle  m_nh;
  ros::Subscriber  m_pos_sub; 
  ros::Subscriber  m_t_pos_sub;

};

#endif // APRIL_TAGS_TURTLEBOT_CONTROLLER_H_
