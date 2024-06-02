# pca_ros

pca9685 driver for ros2 humble

lib needed to install :

sudo pip3 install adafruit-pca9685

sudo pip3 install adafruit-circuitpython-servokit

Huge thanks to https://github.com/stevej52 for the code 

usage:

mkdir pca_ws

cd pca_ws

mkdir src

cd src

git clone https://github.com/0jeetparmar0/pca_ros.git

cd ..

colcon build  or colcon build --symlink-install

start the node 

ros2 run pca_ros pca_node
