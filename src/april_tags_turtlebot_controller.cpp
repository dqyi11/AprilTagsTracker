#include <iostream>
#include "apriltags_tracker/april_tags_turtlebot_controller.h"

using namespace std;

AprilTagsTurtlebotController::AprilTagsTurtlebotController() {

  m_pos_sub = m_nh.subscribe("/april_tag_pos", 1000, &AprilTagsTurtlebotController::positionCallback, this);
  m_t_pos_sub = m_nh.subscribe("/target_pos", 1000, &AprilTagsTurtlebotController::targetPositionCallback, this);

}

AprilTagsTurtlebotController::~AprilTagsTurtlebotController() {

}
  
void AprilTagsTurtlebotController::positionCallback(const apriltags_tracker::april_tag_pos::ConstPtr& msg) {
   std::cout << "ID(" << msg->id << ")=[" << msg->x << " " << msg->y << " " << msg->orientation << "]"; 
}

void AprilTagsTurtlebotController::targetPositionCallback(const apriltags_tracker::target_pos::ConstPtr& msg) {
   std::cout << "ID(" << msg->id << ")=[" << msg->t_x << " " << msg->t_y << " " << msg->t_orientation << "]"; 
}
