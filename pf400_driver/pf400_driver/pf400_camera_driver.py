import cv2

from time import sleep

from threading import Thread

class PF400_CAMERA():

    def __init__(self, robot_connection):

        self.pf400 = robot_connection
        self.scanner_1 = cv2.VideoCapture(2)
        # self.scanner_1.open("usb-046d_HD_Pro_Webcam_C920_806F6D8F-video-index0")
        self.detector_1 = cv2.QRCodeDetector()
        self.scanner_2 = cv2.VideoCapture(0)
        # self.scanner_2.open("usb-046d_HD_Pro_Webcam_C920_B11F1D8F-video-index0")
        self.detector_2 = cv2.QRCodeDetector()
        
        self.stop_camera = False

        self.cam_left_qr_name = None
        self.cam_right_qr_name = None

        # Store joint angles
        # Make sure linear axis lenght is removed from the x axis 

        self.locations = {"Sciclops": [222.0, -38.068, 335.876, 325.434, 79.923, 995.062], 
                          "OT2_Alpha": [243.034, -31.484, 276.021, 383.640, 124.807, -585.407],
                          "OT2_Betha": [163.230, -59.032, 270.965, 415.013, 129.982, -951.510],
                          "Sealer": [201.128, -2.814, 264.373, 365.863, 79.144, 411.553],
                          "Peeler": [262.550, 20.608, 119.290, 662.570, 0.0, 0],
                          "Azenta": [201.128, -2.814, 264.373, 365.863, 79.144, 411.553],
                          "Hidex": [262.550, 20.608, 119.290, 662.570, 0.0, 0],
                          "Biometra": [247.0, 40.698, 38.294, 728.332, 123.077, 301.082]}
        self.module_list = []
        self.robot_reach = 753.0
        self.module_lenght = 685.8      
        # TODO: TABLE LENGHT IS MORE THAN ARM REACH. FIND THE FURTHEST REACH AND ADD THE RAIL LENGHT ON TOP TO FILL THE GAP 685.8
        # self.start_location = self.neutral_joints 
        self.pf400.move_all_joints_neutral()

        self.start_location = [- 990, -330, 400, 990]

    def scan_qr_code(self):  
        i =0
        while i <8 :  
            sleep(0.5)
            ret_1, frame_1 = self.scanner_1.read()
            ret_2, frame_2 = self.scanner_2.read()

            # Display the resulting frame
            # cv2.imshow('frame', frame)
            scanner_1_data, data2, data3 = self.detector_1.detectAndDecode(frame_1)
            scanner_2_data, data2, data3 = self.detector_2.detectAndDecode(frame_2)
            print("1:", scanner_1_data, "2: ",scanner_2_data)
            if scanner_1_data:
                self.cam_left_qr_name = scanner_1_data
            if scanner_2_data:
                self.cam_right_qr_name = scanner_2_data
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            i +=1
    


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

            self.scan_next_row(self.start_location[i])

            if self.cam_left_qr_name not in self.module_list and self.cam_left_qr_name in self.locations.keys():
                self.locations[self.cam_left_qr_name][5] = self.start_location[i]
                self.module_list.append(self.cam_left_qr_name) # Add the module into module list
                left_cam_data = self.cam_left_qr_name
                print(self.locations[self.cam_left_qr_name])

            if self.cam_right_qr_name not in self.module_list and self.cam_right_qr_name in self.locations.keys():
                #TODO:Change this to a function (def reverse_module_location)

                cartesian,phi,rail = self.pf400.forward_kinematics(self.locations[self.cam_right_qr_name])
                        # print(cartesian)
                        # print(cartesian[0] - rail)
                if self.start_location == -990:
                    # TODO: Need to calculate the new location considering the rail location will be at the middle of the module cart. Meaning rail cannot move.
                    pass
                elif self.start_location == 990:
                    # Robot rail at the maximum reach, therefore keep the rail at the same location and calculate the new location with the robot arm only
                    target_on_x_without_rail = cartesian[0] - rail
                    reverse_target_on_x_axis = self.module_lenght - target_on_x_without_rail

                            # print(reverse_target_on_x_axis)
                    cartesian[0] = reverse_target_on_x_axis + self.start_location[i] 
                    cartesian[1] = -cartesian[1] #Switch arm from left to right on y axis
                    cartesian[3] -= 180 
                            # print(cartesian[2])
                            # print(self.start_location[5])
                            # print(cartesian)
                    self.locations[self.cam_right_qr_name] = self.pf400.inverse_kinematics(cartesian_coordinates = cartesian, phi = phi, rail = self.start_location[i])
                    pass
                else:
                    target_on_x_without_rail = cartesian[0] - rail
                    reverse_target_on_x_axis = self.module_lenght - target_on_x_without_rail
                    rail_travel = reverse_target_on_x_axis - target_on_x_without_rail # Find the lenght in between new target location and old target location
                    #Only the rail location will change to move the new target location. Isolated rail location at origin is considered 0. Robot can move to new location by only changing rail location.
                    total_rail_travel = rail_travel + self.start_location[i] 

                            # print(reverse_target_on_x_axis)
                    cartesian[0] = target_on_x_without_rail + total_rail_travel # Keeping the same x axis value while setting a new value to rail to move to the new location.
                    cartesian[1] = -cartesian[1] #Switch arm from left to right on y axis
                    cartesian[3] -= 180 
                            # print(cartesian[2])
                            # print(self.start_location[5])
                            # print(cartesian)
                    self.locations[self.cam_right_qr_name] = self.pf400.inverse_kinematics(cartesian_coordinates = cartesian, phi = phi, rail = total_rail_travel)
                    
                        # print(self.locations[self.cam_right_qr_name][0])

                self.locations[self.cam_right_qr_name][5] = self.start_location[i]
                self.module_list.append(self.cam_right_qr_name) # Add the module into module list
                right_cam_data = self.cam_right_qr_name
                print(self.locations[self.cam_right_qr_name])
            

        print("Workcell exploration completed")

        return self.module_list

    def scan_next_row(self, rail_loc=0.0):

        # Move to next row
        self.pf400.move_one_joint(6,rail_loc, 2)
        sleep(3)
        # Scan the next row
        self.scan_qr_code()

        print(self.cam_left_qr_name)
    
    def calctulate_module_location(self, target_loc, y_direction = 1, reverse_x = 0.0, offset_y = 0.0):
        pass

        
if __name__ == "__main__":
    connection = PF400()
    cam = PF400_CAMERA(connection)
    cam.explore_workcell()
    print(cam.module_list)
    


