import sys, time
import serial
import lewansoul_lx16a
from pynput import keyboard
# try:
#     import keyboard
# except:
#     print("not Root")

from numpy import interp
import numpy as np
import matplotlib.pyplot as plt
class CameraMount:
    def __init__(self,SERIAL_PORT = '/dev/ttyUSB0'):
        self.__ctrl = lewansoul_lx16a.ServoController(
            serial.Serial(SERIAL_PORT, 115200, timeout=1),
        )
        self.SERIAL_PORT = SERIAL_PORT
        self.thresh = 3

    def set_pos(self,desired_pos,velocity=[150,150]):
        # desired_pos = self.to_val(desired_pos)
        velocity = self.pos_vel(desired_pos,velocity)
        self.set_vel(1,velocity[0])
        self.set_vel(2,velocity[1])
       
        while not self.pos_reached(desired_pos): 
            error = self.get_error(desired_pos)
            curr_pos = self.get_pos()
            
            # if curr_pos[0] < 0 or curr_pos[1] < -189:

            #     break
            if abs(error[0]) < self.thresh:
                 self.set_vel(1,0)
            if abs(error[1]) < self.thresh:
                self.set_vel(2,0)
            print("Moving towars desired position: "+str(desired_pos))
            print("Current Pos: " + str(self.to_angle(self.get_pos())))

            print("Current Vel: " + str(velocity))
            print("Error: "+str(error))
        self.set_vel(1,0)
        self.set_vel(2,0)
           
        print("Pos Reached")
       
        
    
    def set_vel(self,id,velocity):
        self.__ctrl.set_motor_mode(id,velocity)
    def get_error(self,desired_pos):
        curr_pos = self.to_angle(self.get_pos())
        return [desired_pos[0] - curr_pos[0],desired_pos[1] - curr_pos[1]]
    def fix_val(self,val):
        max_val = 1193
        if val[0] >= 1071:
            val[0] = -194 + (-1*(max_val-val[0]))
    
        if val[1] >= 1013:
            val[1] = -194 + (-1*(max_val-val[1]))
        return val
    def break_val(self,val):
        max_val = 1193
        min_val = -194
        if val[0] < min_val:
            val[0] = max_val - (abs(val[0]) - abs(min_val))
        if val[1] < min_val:
            val[1] = max_val - (abs(val[1]) - abs(min_val))
        return val
        
    def get_angle(self,val):
        
        return interp(val,[0,1000],[0,240])
    def get_value(self,angle):
        return int(interp(angle,[0,240],[0,1000]))
    def to_angle(self,pos):
        angle = [0] * 2
        pos = self.fix_val(pos)
        if pos[0] <= -60:
             angle[0] = interp(pos[0],[-316,-60],[-90,0])
        if pos[0] >= -60:
            angle[0] = interp(pos[0],[-60,320],[0,90])
        if pos[1] <= -128:
            angle[1] = interp(pos[1] ,[-374,-128],[-90,0])
        if pos[1] >= -128:
            angle[1] = interp(pos[1] ,[-128,244],[0,90])
        
        return angle
    def to_val(self,angle):
        pos = [0] * 2
        if angle[0] < 0:
           angle[0] = interp(angle[0],[-90,0],[-316,-60])
        if angle[0] > 0:
            pos[0] = interp(angle[0],[0,90],[-60,320])
        if angle[1] < 0:
            pos[1] = interp(angle[1],[-90,0],[-374,-128])
        if pos[1] > 0:
            angle[1] = interp(angle[1],[0,90],[-128,244])
        return self.break_val(pos)


        return [self.get_value(pos[0]), self.get_value(pos[1])]
    
    def pos_reached(self,desired_pos):
        error = self.get_error(desired_pos)
        pos_reached = abs(error[0]) <= self.thresh and abs(error[1]) <= self.thresh
        return pos_reached


    def pos_vel(self,desired_pos,vel):
        error = self.get_error(desired_pos)
        vel = [abs(x) for x in vel]
       
        if error[0] < 0:
            print("Velocity 1 Before: " + str(vel[0]))
            vel[0] *= -1 
            print("Velocity 1 After: " + str(vel[0]))
        if error[1] < 0:
            print("Velocity  2 Before: " + str(vel[1]))
            vel[1] *= -1 
            print("Velocity  2 After: " + str(vel[1]))
       

        return vel
        
    def on_press(self,key):
        speed = 200
        try:
            k = key.char  # single-char keys
        except:
            k = key.name  # other keys
        if k == "a":
                print("Left")
                self.set_vel(1,-speed)
        if k == "d":
            self.set_vel(1,speed)
        if  k != "a" and   k != "d":
            self.set_vel(1,0)
        
        if  k == "w":
            self.set_vel(2,-speed)

        if  k =="s":
            self.set_vel(2,speed)
        if  k != "w" and  k != "s":
            self.set_vel(2,0)
    
        if  k =="n":
            self.set_vel(1,0)
            self.set_vel(2,0)
    

    def on_release(self,key):
        try:
            k = key.char  # single-char keys
        except:
            k = key.name  # other keys
        if k == "a":
            self.set_vel(1,0)
        if k == "d":
            self.set_vel(1,0)     
        if  k == "w":
            self.set_vel(2,0)

        if  k =="s":
            self.set_vel(2,0)
       
    
        if k == "n":
            self.set_vel(1,0)
            self.set_vel(2,0)
        if key == keyboard.Key.esc:
            # Stop listener
            return False
    def keyboard_control(self):
       
        listener = keyboard.Listener(
        on_press=self.on_press,on_release=self.on_release)
        listener.start()
        while listener.running:
            print("Recording input")
            

            

    def get_pos(self):
        pos_1 = self.__ctrl.get_position(1)
        pos_2 = self.__ctrl.get_position(2)
        return [pos_1,pos_2]
    def diagnostic(self,save=False):
        user_input = input("Enter desired position: ")
        pos_1_p1 = []
        pos_2_p1 = []
        pos_1_p2 = []
        pos_2_p2 = []
        while True:
            pos = self.get_pos()
            if "," in user_input:
                pos = [int(val) for val in list(user_input.split(",")) ]
                self.set_pos(pos)
                user_input = "None"
            elif  user_input == "Log":
                print("Current Angle: " + str(self.to_angle(self.get_pos())))
                print("Current Pos: " + str(self.get_pos()))
            elif user_input == "Log 1 p1":
                pos_1_p1.append(pos[0])
                print("Pose 1: "+str(pos[0]))
            elif user_input == "Log 2 p1":
                pos_2_p1.append(pos[1])
                print("Pose 2: "+str(pos[1]))
            elif user_input == "Log 1 p2":
                pos_1_p2.append(pos[0])
                print("Pose 1: "+str(pos[0]))
            elif user_input == "Log 2 p2":
                pos_2_p1.append(pos[1])
                print("Pose 2: "+str(pos[1]))
               
                

            # if keyboard.is_pressed("x"):
            #     break
            # if keyboard.is_pressed("t"):
            #     user_input = input("Enter desired position: ")
            #     user_input = user_input.replace('t','')
        if save:
            np.save("./Poses/pos_1_p1",pos_1_p1)
            np.save("./Poses/pos_2_p1",pos_2_p1)
            np.save("./Poses/pos_1_p2",pos_1_p2)
            np.save("./Poses/pos_2_p2",pos_2_p2)
    



        
# if __name__ == '__main__':
#     mount = CameraMount()
#     mount.diagnostic()
#     # pos_1_p1 = np.load("Poses/pos_1_p1.npy")
#     # pos_2_p1 = np.load("Poses/pos_2_p1.npy")
#     # pos_1_p2 = np.load("Poses/pos_1_p2.npy")
#     # pos_2_p2 = np.load("Poses/pos_2_p2.npy")
#     # plt.figure()
#     # plt.plot(pos_1_p1)
#     # plt.figure()
#     # plt.plot(pos_2_p1)
#     # plt.figure()
#     # plt.plot(pos_1_p2)
#     # plt.figure()
#     # plt.plot(pos_2_p2)
#     # plt.show()
        
       

    # mount.keyboard_control()
    