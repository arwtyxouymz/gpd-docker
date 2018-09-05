# hayate_gpd
## Build Docker Image
 `$ make build`
## RUN Docker
 `$ make run`

## Description
 - We use gpd/launch/hsr_15_channels.launch launch file for launching  
       - If you want to change parameters for inference, please look at this launch file.
       - e.g. change the name of subscribing point cloud or type etc...
   - Launch file call 2 programs
       - gpd/src/nodes/grasp_detection_node_server.cpp
         - Service server program written in C++, server for `CPPgpd` service
       - hayate_gpd/scripts/gpd_service_proxy.py 
         - Proxy program for ROS service. client for `CPPgpd`, server for `/hayate/fine_localization` 
   - These program provides ROS service `/hayate/fine_localization`, which is type of FineLocalization.srv
   - If there are no graspable point, this service returns is_success=False
   - Example client for the service is written in gpd/scripts/client_gpd.py 
   - We can test the service by command "rosservice call /hayate/fine_localization" in an environment with FineLocalization.srv
   - Environment variable such as ROS_IP and ROS_MASTER_URI is written in env.list, which is loaded in makefile. If you want to change these, please have a look at Dockerfile.
