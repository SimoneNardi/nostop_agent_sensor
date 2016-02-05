
# bash  file to mv into main folder (/home/pi)
# use ./catkin_make.bash if there are nostop_agent_sensor pkg changes


#!/bin/bash

cd
cd ros_catkin_ws
catkin_make
cd devel/lib/nostop_agent_sensor    # cd to the directory with your node
sudo chown root:root agent_sensor  # change ownship to root
sudo chmod a+rx agent_sensor       # set as executable by all
sudo chmod u+s agent_sensor        # set the setuid bit

sudo chown root:root roomba_agent_sensor  # change ownship to root
sudo chmod a+rx roomba_agent_sensor       # set as executable by all
sudo chmod u+s roomba_agent_sensor        # set the setuid bit
