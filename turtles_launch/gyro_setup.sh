#!/bin/bash


rosservice call /uc_bridge/toggle_daq False

rosservice call /uc_bridge/set_imu 4 0 250 1

rosservice call /uc_bridge/toggle_daq True

rosservice call /uc_bridge/get_imu_info

