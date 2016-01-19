#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include "apriltags_tracker/april_tags_turtlebot_controller.h"
#include <time.h>

#define APRIL_TAG_POS_MSG_NAME      "/april_tag_pos"
#define TARGET_POS_MSG_NAME         "/target_pos"
#define TURTLEBOT_CMD_VEL_MSG_NAME  "/cmd_vel_mux/input/teleop"
#define TURTLEBOT_NAVI_MSG_NAME     "/cmd_vel_mux/input/navi"

using namespace std;

static clock_t last_tick;

#define PI 3.1415926

float convRadius(float radius) {
  if( radius < 0 ) {
    radius = 2*PI + radius;
  }
  return radius;
}

float getDelta(float target, float current) {
  float delta = target - current;
  if( delta  < -PI ){
     delta = 2*PI + delta;
  }
  return delta;
}

AprilTagsTurtlebotController::AprilTagsTurtlebotController() :
  m_max_linear_speed( 0.2f ), 
  m_min_positive_linear_speed( 0.02f ), 
  m_min_linear_speed( -0.2f ), 
  m_max_negative_linear_speed( -0.02f ), 
  m_max_angular_speed( 2.0f ), 
  m_min_positive_angular_speed( 0.15f ), 
  m_min_angular_speed( -2.0f ), 
  m_max_negative_angular_speed( -0.15f ),
  m_ros_rate( 100.0 )
{

  m_target_x = -1.0;
  m_target_y = -1.0;

  m_pos_sub = m_nh.subscribe( APRIL_TAG_POS_MSG_NAME, 1000, &AprilTagsTurtlebotController::positionCallback, this);
  m_t_pos_sub = m_nh.subscribe( TARGET_POS_MSG_NAME, 1000, &AprilTagsTurtlebotController::targetPositionCallback, this);
  m_cmd_vel_pub = m_nh.advertise<geometry_msgs::Twist>( TURTLEBOT_CMD_VEL_MSG_NAME, 10);
  last_tick = clock();
}

AprilTagsTurtlebotController::~AprilTagsTurtlebotController() {

}
  
void AprilTagsTurtlebotController::positionCallback(const apriltags_tracker::april_tag_pos::ConstPtr& msg) {
  //std::cout << "ID(" << msg->id << ")=[" << msg->x << " " << msg->y << " " << msg->orientation << "]" << std::endl; 
  if( false == ros::ok() ) {
    ros::shutdown();
  }
 
  //clock_t current_tick = clock();
  //float delta_ms = (float)(current_tick - last_tick)*1000000/CLOCKS_PER_SEC;
  //cout << "DELTA_MS " << delta_ms << endl; 
  //if( delta_ms >= m_ros_rate ) {
    //last_tick = current_tick;
    control( msg->x, msg->y, msg->orientation );
  //}
}

void AprilTagsTurtlebotController::targetPositionCallback(const apriltags_tracker::target_pos::ConstPtr& msg) {
  std::cout << "ID(" << msg->id << ")=[" << msg->t_x << " " << msg->t_y << " " << msg->t_orientation << "]" << std::endl; 
  m_target_x = msg->t_x;
  m_target_y = msg->t_y;
  
}
  
void AprilTagsTurtlebotController::control(float x, float y, float orientation) {

  float linear_vel = 0.0f;
  float angular_vel = 0.0f;
  float factor = -2.0f;
  
  if( m_target_x > 0 && m_target_y > 0 ) {
    float target_orientation = convRadius( atan2( m_target_y-y, m_target_x-x ) );
    float distance = sqrt( pow(x-m_target_x,2) + pow(y-m_target_y,2) );

    float delta_angular = getDelta( target_orientation ,  orientation );
    angular_vel = factor * delta_angular;
    cout << "TO " << target_orientation << " O " << orientation << " A " << angular_vel << endl;
    if( angular_vel > 0 ) {
      angular_vel = max( min(angular_vel, m_max_angular_speed), m_min_positive_angular_speed ); 
    }
    else {
      angular_vel = min( max(angular_vel, m_min_angular_speed), m_max_negative_angular_speed );
    }
    if( distance >  5 ) {
      if( abs( delta_angular ) < 0.1 ) {
        linear_vel = 0.25;
      } 
      else {
        linear_vel = 0.05;
      }
    }
    cout << "ANGULAR(" << angular_vel <<") VELOCITY(" << linear_vel << ")" << endl;
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
