#!/bin/bash

export ROS_MASTER_URI=http://stick-daisuke.local:11311

roslaunch rc_image_w_tf collect_train_data.launch

