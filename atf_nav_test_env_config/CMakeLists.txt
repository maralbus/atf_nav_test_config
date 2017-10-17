cmake_minimum_required(VERSION 2.8.3)
project(atf_nav_test_env_config)

find_package(catkin REQUIRED COMPONENTS roslaunch rostest)

catkin_package()

set(envlist
    empty
    saturn-ingolstadt
    saturn-lb
    line_passage
    line_passage_obstacle
    narrow_passage_2_cone
    t_passage
    t_passage_obstacle
)

### TEST ###
if(CATKIN_ENABLE_TESTING)
    foreach(env ${envlist}) 
        MESSAGE("Testing env: ${env}")
        roslaunch_add_file_check(upload_navigation_goals.launch robot_env:=${env})
        roslaunch_add_file_check(upload_navigation_goals.launch ROBOT_ENV=${env})
        roslaunch_add_file_check(upload_object_locations.launch robot_env:=${env})
        roslaunch_add_file_check(upload_object_locations.launch ROBOT_ENV=${env})
        add_rostest(test/check_categories.test ARGS robot_env:=${env})
    endforeach()
endif()

### INSTALL ###
install(FILES
  upload_navigation_goals.launch
  upload_object_locations.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY envs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
