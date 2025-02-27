.PHONY: rm colcon bringup autodrive manualdrive fakenode make_map_simulation make_map savemap autosim autosim_makemap

sim := False

rm:
	rm -rf build install log
colcon:
	colcon build --symlink-install --cmake-args -Wno-dev --packages-select wrg_core
	# chmod +x make.sh
	. ~/.bashrc
bringup:
	ros2 launch turtlebot3_bringup robot.launch.py
gazebo:
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
autodrive:
	ros2 run turtlebot3_gazebo turtlebot3_drive
manualdrive:
	ros2 run turtlebot3_teleop teleop_keyboard
fakenode:
	ros2 launch turtlebot3_fake_node turtlebot3_fake_node.launch.py
make_map:
	ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=$(sim)
savemap:
	ros2 run nav2_map_server map_saver_cli -f ~/turtlebot3_map
naviate_on_map:
	ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=$(sim) map:=$(HOME)/turtlebot3_map.yaml
autosim:
	./make.sh "$(MAKE) gazebo" "$(MAKE) autodrive"
autosim_makemap: 
	./make.sh "$(MAKE) gazebo" "$(MAKE) autodrive" "$(MAKE) make_map sim:=$(sim)"


ros2 service call /gazebo/reset_simulation std_srvs/srv/Empty
