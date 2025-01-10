#!/usr/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
WS_SRC=$SCRIPT_DIR/..

python $WS_SRC/controllers/scripts/joy_tgui.py --ros-args -r __node:=joy_to_target --params-file /root/ros2_ws/install/controllers/share/controllers/config/mit_controller_sim_go2.yaml