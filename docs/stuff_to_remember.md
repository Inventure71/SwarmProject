

If can't connect to optitrack:
roslaunch optitrack_ros_communication optitrack_nodes.launch serverIP:=10.205.3.3 namespace:=natnet_ros clientIP:=10.205.10.254 serverType:=unicast udp_server_ip:=10.205.3.47 number_of_nodes:=4 base_id:=2 base_port:=9876

generic 
roslaunch optitrack_ros_communication optitrack_nodes.launch serverIP:=10.205.3.3 namespace:=natnet_ros clientIP:=10.205.10.254 serverType:=unicast udp_server_ip:=CHANGE THIS number_of_nodes:=4 base_id:=2 base_port:=9876

In general 
sudo systemctl restart optitrack_ros.service


If you can't see stuff remember
- optitrack sends only when stuff changes, when you connect it sends the last frame
- check all ports -10 +10 the one you think it will be