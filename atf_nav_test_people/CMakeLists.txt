cmake_minimum_required(VERSION 2.8.3)
project(atf_nav_test_people)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(PROGRAMS
  src/person_of_interest_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
