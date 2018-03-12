rosbag compress -j $(tree -L 3 -f | grep '.bag' | awk '{print $4}')
