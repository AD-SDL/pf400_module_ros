import cv2

from time import sleep

from threading import Thread
from pf400_driver import PF400

class CAMERA(PF400):

    def __init__(self):

        super().__init__("192.168.50.50", 10100)    

        self.scanner_1 = cv2.VideoCapture(0)
        self.detector_1 = cv2.QRCodeDetector()
        self.scanner_2 = cv2.VideoCapture(1)
        self.detector_2 = cv2.QRCodeDetector()
        
        self.stop_camera = False

        self.cam_left_qr_name = None
        self.cam_right_qr_name = None

        # Store joint angles
        # Make sure linear axis lenght is removed from the x axis 
        # self.locations = {"Sciclops": [262.550, 20.608, 119.290, 662.570, 0.0, 0], 
        #                   "OT2_Alpha": [197.185, 59.736, 90.509, 566.953, 82.069, 0],
        #                   "OT2_Betha": [0,0,0,0,0,0],
        #                   "Sealer": [231.788, -27.154, 313.011, 342.317, 0.0, 0.0],
        #                   "Module1": [262.550, 20.608, 119.290, 662.570, 0.0, 0],
        #                   "Biometra": [0,0,0,0,0,0]}

        self.locations = {"Sciclops": [262.550, 20.608, 119.290, 662.570, 0.0, 0], 
                          "OT2_Alpha": [197.185, 59.736, 90.509, 566.953, 82.069, 0],
                          "OT2_Betha": [0,0,0,0,0,0],
                          "Sealer": [231.788, -27.154, 313.011, 342.317, 0.0, 0.0],
                          "Peeler": [262.550, 20.608, 119.290, 662.570, 0.0, 0],
                          "Azenta": [262.550, 20.608, 119.290, 662.570, 0.0, 0],
                          "Hidex": [262.550, 20.608, 119.290, 662.570, 0.0, 0],
                          "Biometra": [0,0,0,0,0,0]}
        self.module_list = []

        self.module_lenght = 685.8     
        # TODO: TABLE LENGHT IS MORE THAN ARM REACH. FIND THE FURTHEST REACH AND ADD THE RAIL LENGHT ON TOP TO FILL THE GAP 685.8
        self.start_location = self.neutral_joints 

        self.start_location[5] = - 990

    def scan_qr_code(self):  
        i =0
        while i <6 :  
            sleep(0.5)
            ret_1, frame_1 = self.scanner_1.read()
            ret_2, frame_2 = self.scanner_2.read()

            # Display the resulting frame
            # cv2.imshow('frame', frame)
            scanner_1_data, data2, data3 = self.detector_1.detectAndDecode(frame_1)
            scanner_2_data, data2, data3 = self.detector_2.detectAndDecode(frame_2)
            print(scanner_1_data, scanner_2_data)
            if scanner_1_data:
                self.cam_left_qr_name = scanner_1_data
            if scanner_2_data:
                self.cam_right_qr_name = scanner_2_data
                return
 
   
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            i +=1
    
        # self.scanner_1.release()
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

        for i in range(4):

            self.scan_next_row()

            if self.cam_left_qr_name not in self.module_list and self.cam_left_qr_name in self.locations.keys():
                self.locations[self.cam_left_qr_name][5] = self.start_location[5]
                self.module_list.append(self.cam_left_qr_name) # Add the module into module list
                left_cam_data = self.cam_left_qr_name
                print(self.locations[self.cam_left_qr_name])

            if self.cam_right_qr_name not in self.module_list and self.cam_right_qr_name in self.locations.keys():
                #TODO:Change this to a function (def reverse_module_location)

                # cartesian,phi,rail = self.forward_kinematics(self.locations[self.cam_left_qr_name][0])
                #         # print(cartesian)
                #         # print(cartesian[0] - rail)

                # target_on_x_without_rail = cartesian[0] - rail
                # reverse_target_on_x_axis = self.module_lenght - target_on_x_without_rail
                # rail_travel = reverse_target_on_x_axis - target_on_x_without_rail
                # total_rail_travel = rail_travel + self.start_location[5]

                #         # print(reverse_target_on_x_axis)
                # cartesian[0] = target_on_x_without_rail + total_rail_travel # New x axis
                # cartesian[1] = -cartesian[1] #Switch arm from left to right on y axis
                # cartesian[3] -= 180 
                #         # print(cartesian[2])
                #         # print(self.start_location[5])
                #         # print(cartesian)
                # self.locations[self.cam_left_qr_name][0] = self.inverse_kinematics(cartesian_coordinates = cartesian, phi = phi, rail = total_rail_travel)
                
                #        # print(self.locations[self.cam_left_qr_name][0])
                # self.locations[self.cam_left_qr_name][1][0] = 0
                self.locations[self.cam_right_qr_name][5] = self.start_location[5]
                self.module_list.append(self.cam_right_qr_name) # Add the module into module list
                right_cam_data = self.cam_right_qr_name
                print(self.locations[self.cam_right_qr_name])
            
            self.start_location[5] += 660

        print("Workcell exploration completed")

        return self.locations

    def scan_next_row(self):

        # Move to next row
        self.move_joint(self.start_location, 2)
        # Scan the next row
        self.scan_qr_code()

        print(self.cam_left_qr_name)
    
    def calctulate_module_location(self, target_loc, y_direction = 1, reverse_x = 0.0, offset_y = 0.0):
        
        # if reverse_x != 0:
        #     x = reverse_x # Add self.robot_x_offset here ???
        # else:    
        #     x = target_loc[0] 
        # y = (target_loc[1] + offset_y) * y_direction
        # z = target_loc[2]
        # phi = target_loc[3]

        # target_joint_angles = self.inverse_kinematics(x,y,z,phi)
        
        # target_loc[0] = target_loc[2] # Setting z hight as the first joint angle for the tower
        # target_loc[1] = target_joint_angles[0][0]
        # target_loc[2] = target_joint_angles[0][1]
        # target_loc[3] = target_joint_angles[0][2]
        # target_loc[4] = 127.0 # An open gripper position for joint 5
        # target_loc[5] = self.start_location[5] 
        
        # return target_loc
        pass

        
if __name__ == "__main__":

    cam = CAMERA()
    cam.explore_workcell()
    print(cam.module_list)
    


