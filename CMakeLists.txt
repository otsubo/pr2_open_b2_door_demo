cmake_minimum_required(VERSION 2.8.3)
project(pr2_open_b2_door_demo)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_ros
  pcl_conversions
  sensor_msgs
  jsk_recognition_msgs
  visualization_msgs
)

add_message_files(DIRECTORY FILES MarkerArrayStamped.msg)

generate_messages(
  DEPENDENCIES
  visualization_msgs)
# catkin_package(
#   CATKIN_DEPENDS visualization_msgs)


catkin_package()

# include_directories(include ${PCL_INCLUDE_DIRS})
# link_libraries(${PCL_LIBERARIES})

# add_executable(random_sample_consensus src/random_sample_consensus.cpp)
# target_link_libraries (random_sample_consensus ${PCL_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(line_detector src/line_detector.cpp)
target_link_libraries(line_detector ${catkin_LIBRARIES})

add_executable(check_caughted_point src/check_caughted_point.cpp)
target_link_libraries(check_caughted_point ${catkin_LIBRARIES})

# add_executable(message_filter_test src/message_filter_test.cpp)
# target_link_libraries(message_filter_test ${catkin_LIBRARIES})

# add_executable(test src/test.cpp)
# target_link_libraries(test ${catkin_LIBRARIES})
