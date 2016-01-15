#include <iostream>
#include <geometry_msgs/Twist.h>
#include "apriltags_tracker/april_tags_turtlebot_controller.h"

#define APRIL_TAG_POS_MSG_NAME      "/april_tag_pos"
#define TARGET_POS_MSG_NAME         "/target_pos"
#define TURTLEBOT_CMD_VEL_MSG_NAME  "/cmd_vel_mux/input/teleop"

using namespace std;

AprilTagsTurtlebotController::AprilTagsTurtlebotController() {

  m_target_x = -1.0;
  m_target_y = -1.0;

  m_pos_sub = m_nh.subscribe( APRIL_TAG_POS_MSG_NAME, 1000, &AprilTagsTurtlebotController::positionCallback, this);
  m_t_pos_sub = m_nh.subscribe( TARGET_POS_MSG_NAME, 1000, &AprilTagsTurtlebotController::targetPositionCallback, this);
  m_cmd_vel_pub = m_nh.advertise<geometry_msgs::Twist>( TURTLEBOT_CMD_VEL_MSG_NAME, 10);
}

AprilTagsTurtlebotController::~AprilTagsTurtlebotController() {

}
  
void AprilTagsTurtlebotController::positionCallback(const apriltags_tracker::april_tag_pos::ConstPtr& msg) {
  //std::cout << "ID(" << msg->id << ")=[" << msg->x << " " << msg->y << " " << msg->orientation << "]" << std::endl; 
  control( msg->x, msg->y, msg->orientation );
}

void AprilTagsTurtlebotController::targetPositionCallback(const apriltags_tracker::target_pos::ConstPtr& msg) {
  std::cout << "ID(" << msg->id << ")=[" << msg->t_x << " " << msg->t_y << " " << msg->t_orientation << "]" << std::endl; 
  m_target_x = msg->t_x;
  m_target_y = msg->t_y;
  
}
  
void AprilTagsTurtlebotController::control(float x, float y, float orientation) {

  float linear_vel = 0.0;
  float angular_vel = 0.0;
  
  if( m_target_x > 0 && m_target_y > 0 ) {
    float target_orientation = atan2( m_target_y-y, m_target_x-x );
    float distance = sqrt( pow(x-m_target_x,2) + pow(y-m_target_y,2) );
    if( distance > 20 ) {
      linear_vel = 0.05;
    }
    angular_vel = 0.1 * (target_orientation - orientation);

    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = linear_vel;
    vel_cmd.linear.y = 0.0;
    vel_cmd.linear.z = 0.0;
    vel_cmd.angular.x = 0.0;
    vel_cmd.angular.y = 0.0;
    vel_cmd.angular.z = angular_vel; 
    m_cmd_vel_pub.publish( vel_cmd );
  }
}
