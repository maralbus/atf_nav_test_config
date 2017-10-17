cmake_minimum_required(VERSION 2.8.3)
project(atf_nav_test_bringup)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

#############
## Install ##
#############
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/alert_error.py
  scripts/fake_gui.py
  scripts/robot_status_retriever.py
  scripts/start_error_monitor.py
  scripts/start_machine.py
  scripts/state_machine_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(test/roslaunch_check_application.xml)  # testing application launch file
endif()
