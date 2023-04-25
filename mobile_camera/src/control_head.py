#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from control_mount import CameraMount
from ROS_TCP_Endpoint_msgs.msg import *
from scipy.spatial.transform import Rotation as R

import numpy as np
class oculusControl:

  def __init__(self):
    self.headset_sub = rospy.Subscriber("/HMDInfo", HMDInput, self.oculus_pos)
    self.right_controller = rospy.Subscriber("rightControllerInfo", ControllerInput, self.callback_right)
    self.move_servos = False
    self.current_pos = []
    self.start_pos = [0,0,0]
    self.zeroed_pos = []
    self.triggerButton = False
    self.camera = CameraMount()
    self.reverse = False
    self.ang_list = []
  def callback_right(self,data):
    """
        This function subscribes to the rightControllerInfo topic 
        parameters: 
                    data: The postion and rotation the the right controller, as well as the value of each button
                    

        """ 

    self.triggerButton = data.triggerButton
    self.move_servos = data.primaryButton
    if data.secondaryButton:
      self.reverse = not self.reverse
    if self.triggerButton:
        print("Zeroing Headset")
        self.start_pos = self.current_pos
  def check_val(self,data):
    """
        This function checks the incoming angles and resctricsthem to human ranges
        parameters: 
                    data: angle values
                    

        """ 
    if abs(data[0]) < 1:
        data[0] = 0
    if data[0] < -60:
        data[0] = -60
    if data[0] > 60:
        data[0] = 60
    if abs(data[1]) < 1:
        data[1] = 0
    if data[1] < -30:
        data[1] = -30
    
    if data[1] > 30:
        data[1] = 30
    return data
    
    
  def oculus_pos(self,data):
    """
        This function subscribes to the /HMDInfo topic 
        parameters: 
                    data: The position and rotation of the VR headset
                    

        """ 
    self.current_pos = [ data.HMD_rot_x ,data.HMD_rot_y,data.HMD_rot_z,data.HMD_rot_w]
    self.current_pos = R.from_quat(self.current_pos)
    self.current_pos = self.current_pos.as_euler('zyx', degrees=True)
    self.zeroed_pos = [self.start_pos[0]- self.current_pos[0], self.start_pos[1]- self.current_pos[1],self.start_pos[2]-self.current_pos[2]]
   
    
    angles = [self.zeroed_pos[1], self.zeroed_pos[2] *-1]
  
    angles = self.check_val(angles)

    
    
    
    if self.move_servos:
        self.stop = False
        self.ang_list.append(angles)
        
        if len(self.ang_list) == 5:
          control_ang = self.ang_list[(len(self.ang_list)-1)//2]
          self.camera.set_pos(control_ang)#sets the position of the camera mount
          self.ang_list = []
      
    else:
        self.stop = True
        self.camera.set_vel(1,0)
        self.camera.set_vel(2,0)
        


def main(args):
  obc = oculusControl()
  rospy.init_node('control_head', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

    


if __name__ == '__main__':
     main(sys.argv)
  