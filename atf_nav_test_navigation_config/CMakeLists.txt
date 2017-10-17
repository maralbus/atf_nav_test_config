cmake_minimum_required(VERSION 2.8.3)
project(atf_nav_test_navigation_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY robots envs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
