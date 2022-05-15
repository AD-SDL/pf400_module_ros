import os.path
import time
import logging

import zmq
import struct

from pf400_client import PF400

#Log Configuration
file_path = os.path.join(os.path.split(os.path.dirname(__file__))[0]  + '/pf400_logs/ros_listener_logs.log')

logging.basicConfig(filename = file_path, level=logging.DEBUG, format = '[%(levelname)s] [%(asctime)s] [%(name)s] %(message)s', datefmt = '%Y-%m-%d %H:%M:%S')

class PF400_listen(object):
    """
    Python interface to socket interface of  PF400.
    Listens for messges come from ROS Arm Node.

    """
    def __init__(self, host, port):

        self.logger = logging.getLogger("PF400_listen")
        self.logger.addHandler(logging.StreamHandler())

        self.host = host
        self.port = port    
        self.OT2_ID = {"bob":1, "alex":2, "jack":3}
        self.listener(host, port)
        
    def command_handler(self, msg):
        robot = PF400()
        robot.set_comm_mode()
        msg = msg.split("@")

        # Check robot state 
        while robot.check_general_state() == -1:

            self.logger.warn("Robot is not intilized! Intilizing now...")
            output = robot.initialize_robot()

        if len(msg) == 3 and msg[0].lower() == "transfer":
            output = robot.program_robot_target(msg[0],self.OT2_ID[msg[1]],self.OT2_ID[msg[2]])
        elif len(msg) == 2 and msg[0].lower() == "rack":
            output = robot.pick_plate_from_rack(self.OT2_ID[msg[1]])
        elif len(msg) == 1 and msg[0].lower() == "complete":
            output = robot.drop_complete_plate()
        else:
            self.logger.error("User sent invalid command")
            return "Invalid command requested by the client!!!"    

        return output
   

    def listener(self, host, port):
        
        try:
            ctx = zmq.Context()
            sock = ctx.socket(zmq.REP)
            sock.bind("tcp://"+host+":"+ port)
            self.logger.info("Starting the command transfer listener")

            i = 1
            while True:
                msg = sock.recv_string()
                i += 1
                time.sleep(1)
                if msg != None:
                    msg_output = self.command_handler(msg)
                    sock.send_string(str(msg_output))
                    # sock.send_string(msg_output + '@' + msg_error + '@' + str(msg_returncode))
        
        except struct.error as e:
            self.logger.error('Lost connection from:', sock)
            # sockClient.shutdown(socket.SHUT_RDWR)
            sock.close()

        except KeyboardInterrupt:
            self.logger.warn('Shutting down socket')
            # sockClient.shutdown(socket.SHUT_RDWR)
            sock.close()
            exit()



if __name__ == "__main__":
    robot_listen = PF400_listen("*", "8085")
