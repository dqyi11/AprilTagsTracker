#include <iostream>
#include <geometry_msgs/Twist.h>
#include "apriltags_tracker/april_tags_turtlebot_controller.h"

#define APRIL_TAG_POS_MSG_NAME      "/april_tag_pos"
#define TARGET_POS_MSG_NAME         "/target_pos"
#define TURTLEBOT_CMD_VEL_MSG_NAME  "/cmd_vel_mux/input/teleop"

using namespace std;

AprilTagsTurtlebotController::AprilTagsTurtlebotController() {

  m_pos_sub = m_nh.subscribe( APRIL_TAG_POS_MSG_NAME, 1000, &AprilTagsTurtlebotController::positionCallback, this);
  m_t_pos_sub = m_nh.subscribe( TARGET_POS_MSG_NAME, 1000, &AprilTagsTurtlebotController::targetPositionCallback, this);
  m_cmd_vel_pub = m_nh.advertise<geometry_msgs::Twist>( TURTLEBOT_CMD_VEL_MSG_NAME, 10);
}

AprilTagsTurtlebotController::~AprilTagsTurtlebotController() {

}
  
void AprilTagsTurtlebotController::positionCallback(const apriltags_tracker::april_tag_pos::ConstPtr& msg) {
   std::cout << "ID(" << msg->id << ")=[" << msg->x << " " << msg->y << " " << msg->orientation << "]"; 
}

void AprilTagsTurtlebotController::targetPositionCallback(const apriltags_tracker::target_pos::ConstPtr& msg) {
   std::cout << "ID(" << msg->id << ")=[" << msg->t_x << " " << msg->t_y << " " << msg->t_orientation << "]"; 
}
  
void AprilTagsTurtlebotController::control(float x, float y, float orientation) {

  float linear_vel = 0.0;
  float angular_vel = 0.0;

  geometry_msgs::Twist vel_cmd;
  vel_cmd.linear.x = linear_vel;
  vel_cmd.linear.y = 0.0;
  vel_cmd.linear.z = 0.0;
  vel_cmd.angular.x = 0.0;
  vel_cmd.angular.y = 0.0;
  vel_cmd.angular.z = angular_vel; 
  m_cmd_vel_pub.publish( vel_cmd );
}
