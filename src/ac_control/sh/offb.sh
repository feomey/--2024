#!/bin/bash
source ~/.bashrc
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4 mavros_posix_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; source /home/zhu/acfly/my_code/devel/setup.bash; rosrun ac_control offb_cir; exec bash"' \
