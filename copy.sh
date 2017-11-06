echo 'Copying files'
echo -ne '\033[92m####                                       (10%)\033[0m\r'
cp -R atf_nav_test_bringup/launch $(rospack find msh_bringup)/
echo -ne '\033[92m########                                   (20%)\033[0m\r'
cp -R atf_nav_test_env_config/envs $(rospack find msh_env_config)/
echo -ne '\033[92m############                               (30%)\033[0m\r'
cp -R atf_nav_test_gazebo_worlds/launch $(rospack find msh_gazebo_worlds)/
echo -ne '\033[92m################                           (40%)\033[0m\r'
cp -R atf_nav_test_gazebo_worlds/Media/models $(rospack find msh_gazebo_worlds)/Media/
echo -ne '\033[92m####################                       (50%)\033[0m\r'
cp -R atf_nav_test_gazebo_worlds/urdf $(rospack find msh_gazebo_worlds)/
echo -ne '\033[92m########################                   (60%)\033[0m\r'
cp -R atf_nav_test_navigation_config/envs $(rospack find msh_navigation_config)/
echo -ne '\033[92m############################               (70%)\033[0m\r'
cp -R atf_nav_test_gazebo_worlds/launch $(rospack find cob_gazebo_worlds)/
cp -R atf_nav_test_gazebo_worlds/objects $(rospack find cob_gazebo_objects)/
cp atf_nav_test_gazebo_worlds/Media/models/box_1x1x1.STL $(rospack find cob_gazebo_objects)/Media/models
echo -ne '\033[92m################################           (80%)\033[0m\r'
cp -R atf_nav_test_gazebo_worlds/Media/models $(rospack find cob_gazebo_worlds)/Media/
cp catkin_make.sh $(rospack find atf_nav_test)/../../
echo -ne '\033[92m####################################       (90%)\033[0m\r'
cp -R atf_nav_test_gazebo_worlds/urdf $(rospack find cob_gazebo_worlds)/
echo -ne '\033[92m########################################   (100%)\033[0m\r'
echo -ne '\n'
