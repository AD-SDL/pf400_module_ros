import socket
import time



def connect_robot():
    #create an INET, STREAMing socket (IPv4, TCP/IP)
    try:
        PF400 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except socket.error:
        print('Failed to create socket')

    print('Socket Created')

    ROBOT_IP= "192.168.0.1"
    ROBOT_PORT = 10100
    #Connect the socket object to the robot using IP address (string) and port (int)
    PF400.connect((ROBOT_IP,ROBOT_PORT))

    print('Socket Connected to ' + ROBOT_IP + " Port:", ROBOT_PORT )
    #TODO:Read the status of the robot

    return PF400

def disconnect_robot(PF400):
    PF400.close()
    print("TCP/IP client is closed")

def enable_power():

    PF400 = connect_robot()

    #Send cmd to Activate the robot
    command = 'hp 1\n'
    # Add ASCII NULL character at the end of the cmd string
    try:
        PF400.send(bytes(command.encode('ascii')))
        # time.sleep(15)
        out_msg = PF400.recv(4096).decode("utf-8")
        print(out_msg)
    except socket.error:
        print('Failed to send data')
    disconnect_robot(PF400)    

def attach_robot():
    PF400 = connect_robot()

    #Send cmd to Activate the robot
    command = 'attach 1\n'
    # Add ASCII NULL character at the end of the cmd string
    try:
        PF400.send(bytes(command.encode('ascii')))
        # time.sleep(15)
        out_msg = PF400.recv(4096).decode("utf-8")
        print(out_msg)
    except socket.error:
        print('Failed to send data')
    disconnect_robot(PF400)    
    
def home_robot():

    PF400 = connect_robot()

    #Send cmd to Activate the robot
    command = 'home\n'
    # Add ASCII NULL character at the end of the cmd string
    try:
        PF400.send(bytes(command.encode('ascii')))
        # time.sleep(15)
        out_msg = PF400.recv(4096).decode("utf-8")
        print(out_msg)
    except socket.error:
        print('Failed to send data')
    disconnect_robot(PF400)    

def set_speed(speed):
    PF400 = connect_robot()

    #Send cmd to Activate the robot
    command = "mspeed" + str(speed) +"\n"
    # Add ASCII NULL character at the end of the cmd string
    try:
        PF400.send(bytes(command.encode('ascii')))
        # time.sleep(15)
        out_msg = PF400.recv(4096).decode("utf-8")
        print(out_msg)
    except socket.error:
        print('Failed to send data')
    disconnect_robot(PF400) 

def locate_robot():
    
    PF400 = connect_robot()
    #Send cmd to Activate the robot
    command = 'where\n'
    # Add ASCII NULL character at the end of the cmd string
    try:
        PF400.send(bytes(command.encode('ascii')))
        # time.sleep(15)
        # data = bytearray()
        # while True:
        #     packet = PF400.recv(4096)
        #     if not packet:
        #         break
        #     data.extend(packet)
        out_msg = PF400.recv(8000).decode("utf-8")
    except socket.error:
        print('Failed to send data')

    disconnect_robot(PF400)

    return out_msg



if __name__ == "__main__":
    attach_robot()
    location = locate_robot()
    print(location)
