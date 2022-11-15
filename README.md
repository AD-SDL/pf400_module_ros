# pf400_driver
A repository for PF400 Collaborative Robot Arm driver, including user manuals and remote control interfaces.

## PF400 Remote Client
Python interface that allows remote commands to be executed using simple string messages over telnet socket on PF400 cobot.  `pf400_module/pf400_driver/pf400_driver/pf400_driver.py`

- PF400 is the main object that will be used for operations such as remote connection as well as sending movement commands.
- Programs are sent to the 192.168.50.50 IP address and 10x00 port numbers (first robot port number: 10100). 
- A program sent to robot will be executed immediately unless there is a prior operation running on the robot. 
- If a second motion command is sent while the referenced robot is moving, the second command is blocked and will not reply until the first motion is complete.
## pf400_camera_driver 

This is a sub class of the PF400 class, which includes more specific functions that will be utilized only in Rapid Prototyping Lab. `/pf400_module/pf400_driver/pf400_driver/pf400_camera_driver.py`

- This class is designed to move PF400 robot in the workcell and also discovere the module location by using two cameras with OpenCV algorithms.

### Current features
* Robot initilazation (enable power, home robot joints, attach robot to the software and check robot state)
* Robot Forward & Inverse kinematic calcuations to calculate desired locations and switch between cooardinate systems
* Plate transfer between two locations
* Plate rotation to change the grab angle between wide and narrow
* Remove & Replace plate lid
* Workcell discovery to locate module locations 
## pf400_client 
This a ROS2 wrapper that accepts service calls from wei_client with string messages to execute transfers between source and target locations.

# Development
## Enable remote connections on PF400
- Enter IP address of the PF400 (192.168.50.50) in a web browser and then clink on Admin.
- Go to startup configuration under wizards and setup tools. Make sure that "Tcp_cmd_server_pa" project is loaded to be automatically compiled when the robot is turned on.
- Go to Control Panels and then Operator Control panel. Verify that TCP Command Server is running. 

![Control Panel TCP Server](https://github.com/AD-SDL/PF400_cobot/blob/master/resources/diagrams-figures/control-panel.png)

- To free the robot joints manualy, stop the TCP server and go to Virtual Panel and choose free all joints, click plus.

![Free Joints](https://github.com/AD-SDL/PF400_cobot/blob/master/resources/diagrams-figures/free-joint-mode.png)

## ROS Install
- `cd ~`
- `source /opt/ros/foxy/setup.bash`
- `mkdir -p ~/pf400_ws/src`
- `cd ~/pf400_ws/src`
- `git clone https://github.com/AD-SDL/pf400_module.git`
- `cd ~/pf400_ws`
- `sudo apt install python3-rosdep2`
- `rosdep update && rosdep install -i --from-path src`
- `sudo apt install python3-colcon-common-extensions`
- `colcon build`
- `source install/setup.bash`

## Python Driver Install

- `conda create -n rpl-test python=3.8`
- `conda activate rpl-test`
- `git clone https://github.com/AD-SDL/pf400_module.git`
- `cd pf400_driver`
- `#pip install -r requirements.txt`
- `pip install -e .`

Better to install in develop-mode while the config is still changing

## Rviz Visualization
### Launch with real robot
- `ros2 launch pf400_description pf400_rviz.launch.py`
### Launch with fake robot hardware
- `ros2 launch pf400_description pf400_rviz.launch.py fake_hardware:=True`

![RViz PF400 Visualization](https://github.com/AD-SDL/PF400_cobot/blob/master/resources/diagrams-figures/pf400-rviz.png)
### Launch with real robot
- `ros2 launch pf400_description pcr_workcell.launch.py`
### Launch with fake robot hardware
- `ros2 launch pf400_description pcr_workcell.launch.py fake_hardware:=True`

![RViz PCR Workcell Visualization](https://github.com/AD-SDL/PF400_cobot/blob/master/resources/diagrams-figures/pcr_workcell.png)

## Create new configuration

## Logging
CURRENTLY DEFECTIVE
- Logs go to the "running" PC `/pf400_logs/robot_client_logs.log`

## Resources

* Original documents `/resources/documents`
* Serverside VBA software `/resources/original_tcp_code`
* Backup config files `/resources/config_files`

## TODO

* move log to receive arbitrary path
* move log to receive robot_name
* move log to receive date
