import cv2

from time import sleep

from threading import Thread
from pf400_driver.pf400_driver import PF400
qr_name = "TEST"

class CAMERA(PF400):

    def __init__(self):

        super().__init__("192.168.50.50", 10100)    

        self.scanner = cv2.VideoCapture(0)
        self.detector = cv2.QRCodeDetector()
        (self.grabbed, self.frame) = self.scanner.read()
        self.stop_camera = False

        self.cam_left_qr_name = None
        self.cam_right_qr_name = None

        # Store joint angles
        # Make sure linear axis lenght is removed from the x axis 
        self.locations = {"sciclops": [[262.550, 20.608, 119.290, 662.570, 0.0, 0],[-1]], 
                          "ot2_1": [[197.185, 59.736, 90.509, 566.953, 82.069, 0],[-1]],
                          "ot2_2": [[0,0,0,0,0,0],[-1]],
                          "sealer": [[231.788, -27.154, 313.011, 342.317, 0.0, 0.0],[-1]],
                          "Module1": [[0,0,0,0,0,0],[-1]],
                          "thermocycler": [[0,0,0,0,0,0],[-1]]}
        self.module_lenght = 500.0

        self.start_location = self.get_joint_states()
        self.start_location[5] = -990

    def scan_qr_code(self):  
        i =0
        while i <6 :  
            sleep(0.5)
            ret, frame = self.scanner.read()
            # Display the resulting frame
            cv2.imshow('frame', frame)
            data1, data2, data3 = self.detector.detectAndDecode(frame)
            
            if data1:
                self.cam_left_qr_name = data1
                return
   
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            i +=1
    
        # self.scanner.release()
        # Destroy all the windows
        # cv2.destroyAllWindows()

    def explore_workcell(self):

        # TODO: Make sure that robot stops are always at the module origins
        # TODO: Find the target locations with respect to the origin locations
        # TODO: In defult locations use joint angles then update them with new joint angle locations
        # TODO: Assume that the defult locations are taken when all the modules where left side of the PF400
        # TODO: Find module lenght and update it in the code
        # TODO: Figure out how to deal with rotation offset on the -180 rotation

        left_cam_data = self.cam_left_qr_name
        right_cam_data = self.cam_right_qr_name
        self.move_all_joints_neutral()

        for i in range(4):

            self.scan_next_row()

            if self.cam_left_qr_name != left_cam_data and self.cam_left_qr_name in self.locations.keys():
                self.locations[self.cam_left_qr_name][0][5] = self.start_location[5]
                self.locations[self.cam_left_qr_name][1][0] = 0 # Zero means this modules is found in the workcell
                left_cam_data = self.cam_left_qr_name
                print(self.locations[self.cam_left_qr_name][0])

            if self.cam_right_qr_name != right_cam_data and self.cam_right_qr_name in self.locations.keys():
                #TODO:Change this to a function (def reverse_module_location)

                cartesian,phi,rail = self.forward_kinematics(self.locations[self.cam_right_qr_name][0])
                print(cartesian)
                # print(cartesian[0] - rail)
                reverse_target_on_x_axis = self.module_lenght - (cartesian[0] - rail)
                # print(reverse_target_on_x_axis)
                cartesian[0] = reverse_target_on_x_axis + self.start_location[5] # New x axis
                cartesian[1] = -cartesian[1] #Switch arm from left to right on y axis
                cartesian[3] -= 180 
                # print(cartesian[2])
                self.locations[self.cam_right_qr_name][0] = self.inverse_kinematics(cartesian, phi, self.start_location[5])
                print(self.locations[self.cam_right_qr_name][0])
                self.locations[self.cam_right_qr_name][1][0] = 0
                
                right_cam_data = self.cam_right_qr_name

            self.start_location[5] += 660

        print("Workcell exploration completed")

        return self.locations

    def scan_next_row(self):

        # Move to next row
        self.send_command(self.create_move_joint_command(self.start_location, 2, True, False))
        # Scan the next row
        self.scan_qr_code()

        print(self.cam_left_qr_name)
    
    def calctulate_module_location(self, target_loc, y_direction = 1, reverse_x = 0.0, offset_y = 0.0):
        
        if reverse_x != 0:
            x = reverse_x # Add self.robot_x_offset here ???
        else:    
            x = target_loc[0] 
        y = (target_loc[1] + offset_y) * y_direction
        z = target_loc[2]
        phi = target_loc[3]

        target_joint_angles = self.inverse_kinematics(x,y,z,phi)
        
        target_loc[0] = target_loc[2] # Setting z hight as the first joint angle for the tower
        target_loc[1] = target_joint_angles[0][0]
        target_loc[2] = target_joint_angles[0][1]
        target_loc[3] = target_joint_angles[0][2]
        target_loc[4] = 127.0 # An open gripper position for joint 5
        target_loc[5] = self.start_location[5] 
        
        return target_loc

        
if __name__ == "__main__":

    cam = CAMERA()
    cam.explore_workcell()
    print(cam.locations)
    


