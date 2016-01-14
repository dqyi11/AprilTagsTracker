# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(apriltags_tracker)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
             roscpp
             sensor_msgs
             cv_bridge
             image_transport
             std_msgs
             message_generation
            )

#find_package(OpenCV 2.4 REQUIRED COMPONENTS core highgui imgproc calib3d
#                                            ml features2d objdetect )
find_package(OpenCV REQUIRED)

add_message_files (
  FILES
  april_tag_pos.msg
  target_pos.msg
)

generate_messages (
  DEPENDENCIES
  std_msgs
)

# Declare this project as a catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello_world
   CATKIN_DEPENDS message_runtime
)


# Add the headers from roscpp
include_directories(include ${catkin_INCLUDE_DIRS}
                            ${OpenCV_INCLUDE_DIRS}
                            /usr/local/include
                            /usr/include/eigen3
                            )

# Define a library target called hello_world
#add_library(hello_world src/hello_world.cpp)
#target_link_libraries(hello_world ${roscpp_LIBRARIES})

add_executable(april_tags_tracker src/april_tags_tracker.cpp
                                  src/apriltags_tracker_demo.cpp
              )
target_link_libraries(april_tags_tracker ${catkin_LIBRARIES} 
                                         ${OpenCV_LIBRARIES}
                                         apriltags
                     )

add_executable(april_tags_turtlebot_controller src/april_tags_turtlebot_controller.cpp
                                               src/apriltags_turtlebot_controller_demo.cpp
              )
target_link_libraries(april_tags_turtlebot_controller ${catkin_LIBRARIES} 
                     )

