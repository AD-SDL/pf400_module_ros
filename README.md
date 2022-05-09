
# PF400_cobot
A repository for PF400 Collaborative Robot Arm documentation, including user manuals and remote control interfaces.
## Remote client
    
Python interface that allows remote commands to be executed using simple string messages over TCP/IP on PF400 cobot.  `/pf400_client/pf400_client.py`

- PF400 is the main object that will be used for operations such as remote connection as well as sending movement commands.
- Programs are sent to the 192.168.0.1 IP address and 10x00 port numbers (first robot port number: 10100). 
- Robot data will be loaded from `/utils/robot_data.json` which contains motion profiles and joint locations.
- A program sent to robot will be executed immediately unless there is a prior operation running on the robot. 
- If a second motion command is sent while the referenced robot is moving, the second command is blocked and will not reply until the first motion is complete.

# Development
## Enable remote connections on PF400
- Enter IP address of the PF400 (192.168.0.1) in a web browser and then clink on Admin.
- Go to startup configuration under wizards and setup tools. Make sure that "Tcp_cmd_server" project is loaded to be automatically compiled when the run is turned on.
- Go to Control Panels and then Operator Control panel. Verify that TCP Command Server is running (see the image). 
(https://raw.githubusercontent.com/AD-SDL/PF400_cobot/blob/master/resources/diagrams-figures/operator-control.PNG)

## Install

    conda create -n rpl-test python=3.8
    conda activate rpl-test

    git clone https://www.github.com/AD-SDL/PF400_cobot
    cd PF400_cobot
    #pip install -r requirements.txt
    pip install -d . 

Better to install in develop-mode while the config is still changing

## Ros Install

## Create new configuration

## Logging

Logs go to the "running" PC `~/.logs_pf400/logs_date.txt`

## Resources

* Original documents `/resources/documents`
* Serverside VBA software `/resources/original_tcp_code`
* Backup config files `/resources/config_files`

## TODO

* move log to receive arbitrary path
* move log to receive robot_name
* move log to receive date
