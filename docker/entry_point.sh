#!/bin/bash
source /opt/ros/humble/setup.bash
# # echo "Provided arguments: $@"
# cd  ~/YDLidar-SDK


# mkdir build
# cd build
# cmake ..
# make
# sudo make install

# cd ..
# pip install .

# sudo chmod 777 /dev/ttyUSB0

cd
cd ~/coop_ws/src
rm -r sim_car/
cd ..

colcon build --symlink-install
source ~/coop_ws/install/setup.bash

tmux
