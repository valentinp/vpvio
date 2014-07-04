#!/bin/bash
myIP=$(ip a s|sed -ne '/127.0.0.1/!{s/^[ \t]*inet[ \t]*\([0-9.]\+\)\/.*$/\1/p}')
export ROS_MASTER_URI="http://"${myIP}":11311/"
export ROS_IP=$myIP
