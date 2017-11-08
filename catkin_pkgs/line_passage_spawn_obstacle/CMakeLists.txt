cmake_minimum_required(VERSION 2.8.3)
project(line_passage_spawn_obstacle)

find_package(catkin REQUIRED COMPONENTS
  atf_core
  roslaunch
  rospy
  rostopic
)
catkin_package(
   CATKIN_DEPENDS rospy
)

###########
## Build ##
###########

#############
## Install ##
#############

install(PROGRAMS scripts/application.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############
atf_test()
