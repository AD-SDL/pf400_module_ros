#! /usr/bin/env python3

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from std_msgs.msg import String
from std_srvs.srv import Empty

from time import sleep
import json

from threading import Thread

from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions 

from pf400_driver.errors import ConnectionException, CommandException
from pf400_driver.pf400_driver import PF400
# from pf400_driver.errors import ConnectionException, CommandException
from pf400_driver.pf400_camera_driver import PF400_CAMERA


"""The server that takes incoming WEI flow requests from the experiment application"""
import json
from argparse import ArgumentParser
from contextlib import asynccontextmanager
import time
from fastapi import FastAPI, File, Form, UploadFile
from fastapi.responses import JSONResponse

workcell = None
global sealer, state
serial_port = '/dev/ttyUSB0'
local_ip = 'parker.alcf.anl.gov'
local_port = '8000'


       
@asynccontextmanager
async def lifespan(app: FastAPI):
    global pf400, state
    """Initial run function for the app, parses the worcell argument
        Parameters
        ----------
        app : FastApi
           The REST API app being initialized

        Returns
        -------
        None"""
    ip = "127.0.0.1"
    port = 8085

    ip= "146.137.240.35"
    port = 10100

    try:
        pf400 = PF400(ip, port)
        pf400.initialize_robot()
        #module_explorer = PF400_CAMERA(pf400)
        state="READY"

    except ConnectionException as error_msg:
        state = "ERROR"
        print(error_msg)
        #get_logger().error(str(error_msg))

    except Exception as err:
        state = "ERROR"
        print(err)
        #get_logger().error(str(err))
    else:
        print("PF400 online")
    yield

    # Do any cleanup here
    pass


app = FastAPI(lifespan=lifespan, )

@app.get("/state")
def state():
    global state
    return JSONResponse(content={"State": state })

@app.get("/description")
async def description():
    global sealer
    return JSONResponse(content={"State": sealer.get_status() })

@app.get("/resources")
async def resources():
    global sealer
    return JSONResponse(content={"State": sealer.get_status() })


@app.post("/action")
def do_action(
    action_handle: str,
    action_vars):
    response = {"action_response": "", "action_msg": "", "action_log": ""}
    print(action_vars)
    global sealer, state
    if state == "PF400 CONNECTION ERROR":
        message = "Connection error, cannot accept a job!"
        #get_logger().error(message)
        #response.action_response = -1
        #wresponse.action_msg= message
        return response

    while state != "READY":
        #get_logger().warn("Waiting for PF400 to switch READY state...")
        sleep(0.2)
  
    #get_logger().info('Received Action: ' + action_handle.upper())
    sleep(0.3) #Before starting the action, wait for stateRefresherCallback function to cycle for at least once to avoid data loss.

    vars = json.loads(action_vars)
    #get_logger().info(str(vars))

    err=False
    state = "BUSY"
    if action_handle == "transfer":

        source_plate_rotation = ""
        target_plate_rotation = ""

        if 'source' not in vars.keys():
            err = True
            msg = "Pick up location is not provided. Canceling the job!"
        elif 'target' not in vars.keys():
            err = True
            msg = "Drop off up location is not provided. Canceling the job!"
        elif len(vars.get('source')) != 6:
            err = True
            msg = "Position 1 should be six joint angles lenght. Canceling the job!"
        elif len(vars.get('target')) != 6:
            err = True
            msg = "Position 2 should be six joint angles lenght. Canceling the job!"

        if err:
            response["action_response"] = -1
            response["action_msg"]= msg
            #get_logger().error('Error: ' + msg)
            state = "ERROR"
            return response

        if 'source_plate_rotation' not in vars.keys():
            #get_logger().info("Setting source plate rotation to 0")
            pass
        else:
            source_plate_rotation = str(vars.get('source_plate_rotation'))

        if 'target_plate_rotation' not in vars.keys():
            #get_logger().info("Setting target plate rotation to 0")
            pass
        else:
            target_plate_rotation = str(vars.get('target_plate_rotation'))

        source = vars.get('source')
        #get_logger().info("Source location: " + str(source))
        target = vars.get('target')
        #get_logger().info("Target location: "+ str(target))
        
        try:
            pf400.transfer(source, target, source_plate_rotation, target_plate_rotation)

        except Exception as err:
            #response.action_msg = "Transfer failed. Error:" + err
            #response.action_response = -1
            if pf400.robot_warning.upper() != "CLEAR":
                #response.action_msg = pf400.robot_warning.upper()
                pass
            state = "ERROR"

        else:    
            #response.action_response = 0
            #response.action_msg = "PF400 succsessfully completed a transfer"
            state = "READY"

        finally:
            #get_logger().info('Finished Action: ' + action_handle)
            return response

    elif action_handle == "remove_lid":

        target_plate_rotation = ""

        if 'target' not in vars.keys():
            err = 1
            msg = "Target location is not provided. Canceling the job!"
            #get_logger().error(msg)
            state = "ERROR"
                

        if len(vars.get('target')) != 6:
            err = 1
            msg = "Target position should be six joint angles lenght. Canceling the job!"
            #get_logger().error(msg)
            state = "ERROR"
        
        if err:
            #response.action_response = -1
            #response.action_msg= msg
            #get_logger().error('Error: ' + msg)
            return response

        if 'target_plate_rotation' not in vars.keys():
            pass
            #get_logger().info("Setting target plate rotation to 0")
        else:
            target_plate_rotation = str(vars.get('target_plate_rotation'))
        
        target = vars.get('target')
        #get_logger().info("Target location: " + str(target))

        lid_height = vars.get('lid_height', 7.0)
        #get_logger().info("Lid hight: " + str(lid_height))
            
        try:
            pf400.remove_lid(target, lid_height, target_plate_rotation)
        except Exception as err:
            #response.action_response = -1
            #response.action_msg= "Remove lid failed. Error:" + err
            state = "ERROR"
        else:    
            #response.action_response = 0
            #response.action_msg= "Remove lid successfully completed"
            state = "READY"

        finally:
            #get_logger().info('Finished Action: ' + action_handle)
            return response
        
    elif action_handle == "replace_lid":

        target_plate_rotation = ""

        if 'target' not in vars.keys():
            err = 1
            msg = "Target location is not provided. Canceling the job!"
            #get_logger().error(msg)
            state = "ERROR"
                

        if len(vars.get('target')) != 6:
            err = 1
            msg = "Target position should be six joint angles lenght. Canceling the job!"
            #get_logger().error(msg)
            state = "ERROR"

        if err:
            #response.action_response = -1
            #response.action_msg= msg
            #get_logger().error('Error: ' + msg)
            return response
    
        if 'target_plate_rotation' not in vars.keys():
            #get_logger().info("Setting target plate rotation to 0")
            pass
        else:
            target_plate_rotation = str(vars.get('target_plate_rotation'))
        

        if 'lid_height' not in vars.keys():
            #get_logger().info('Using defult lid hight')
            lid_height = 7.0

        else:    
            lid_height = vars.get('lid_height')

        #get_logger().info("Lid hight: " + str(lid_height))

        try:    
            pf400.replace_lid(target, lid_height, target_plate_rotation)
        except Exception as err:
            #response.action_response = -1
            #response.action_msg= "Replace lid failed. Error:" + err
            state = "ERROR"
        else:    
            #response.action_response = 0
            #response.action_msg= "Replace lid successfully completed"
            state = "READY"
        finally:
            #get_logger().info('Finished Action: ' + action_handle)
            return response

    else:
        msg = "UNKOWN ACTION REQUEST! Available actions: explore_workcell, transfer, remove_lid, replace_lid"
        #response.action_response = -1
        #response.action_msg = msg
        #get_logger().error('Error: ' + msg)
        state = "ERROR"
        return response


if __name__ == "__main__":
    import uvicorn
    print("asdfsaf")
    uvicorn.run("a4s_sealer_REST:app", host=local_ip, port=local_port, reload=True, ws_max_size=100000000000000000000000000000000000000)

