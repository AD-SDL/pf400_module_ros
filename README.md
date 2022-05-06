# PF400_cobot
A repository for PF400 Collaborative Robot Arm documentation, including user manuals and remote control interfaces.

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
