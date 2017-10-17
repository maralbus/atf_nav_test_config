cmake_minimum_required(VERSION 2.8.3)
project(atf_nav_test_gazebo_worlds)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

### INSTALL ###
install(DIRECTORY Media launch urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch)
