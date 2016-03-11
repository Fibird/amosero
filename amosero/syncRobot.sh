#this scripts syncs the robot with the current catkin workspace source
rsync --progress -r catkin_ws/src/ linaro@10.10.10.100:/home/linaro/catkin_ws/src/