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
  def callback_right(self,data):
    self.triggerButton = data.triggerButton
    self.move_servos = data.primaryButton
    if self.triggerButton:
        print("Zeroing Headset")
        self.start_pos = self.current_pos
  def check_val(self,data):
    if abs(data[0]) < 1:
        data[0] = 0
    if data[0] < -90:
        data[0] = -90
    if data[0] > 90:
        data[0] = 90
    if abs(data[1]) < 1:
        data[1] = 0
    if data[1] < -90:
        data[1] = -90
    if data[1] < -90:
        data[1] = -90
    if data[1] > 90:
        data[1] = 90
    return data
    
    
  def oculus_pos(self,data):
    self.current_pos = [ data.HMD_rot_x ,data.HMD_rot_y,data.HMD_rot_z,data.HMD_rot_w]
    self.current_pos = R.from_quat(self.current_pos)
    self.current_pos = self.current_pos.as_euler('zyx', degrees=True)
    self.zeroed_pos = [self.start_pos[0]- self.current_pos[0], self.start_pos[1]- self.current_pos[1],self.start_pos[2]-self.current_pos[2]]
   
    
    
  
    angles = self.zeroed_pos[0:2][::-1]
    angles = self.check_val(angles)
    # angles[1] = -1* angles[1]
    if self.move_servos:
        self.camera.set_pos(angles)
    else:
        print("Current Pos: "+ str(angles))
        print("Starting Pos: "+ str(self.start_pos))


def main(args):
  obc = oculusControl()
  rospy.init_node('control_head', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

    


if __name__ == '__main__':
     main(sys.argv)
    # mount = CameraMount()
    # mount.diagnostic()
    # mount.keyboard_control()
