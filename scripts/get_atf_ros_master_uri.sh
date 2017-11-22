#!/bin/bash
export ROS_MASTER_URI=$(grep -r "Registering with master node" ~/.ros/log/latest | tail -c 25)
echo $ROS_MASTER_URI

