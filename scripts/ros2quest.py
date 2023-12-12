#!/usr/bin/env python
import rospy
import sys

from quest2ros.msg import OVR2ROSInputs, OVR2ROSHapticFeedback
from geometry_msgs.msg import PoseStamped, Twist 
import numpy as np 



class ros2quest:

  def __init__(self):

    #subscriber to quest
    self.ovr2ros_right_hand_pose_sub = rospy.Subscriber("/ovr2ros_right_hand_pose",PoseStamped,self.ovr2ros_right_hand_pose_callback)
    self.ovr2ros_right_hand_twist_sub = rospy.Subscriber("/ovr2ros_right_hand_twist",Twist,self.ovr2ros_right_hand_twist_callback)
    self.ovr2ros_right_hand_inputs_sub = rospy.Subscriber("/ovr2ros_right_hand_inputs",OVR2ROSInputs,self.ovr2ros_right_hand_inputs_callback)
    self.ovr2ros_left_hand_pose_sub = rospy.Subscriber("/ovr2ros_left_hand_pose",PoseStamped,self.ovr2ros_left_hand_pose_callback)
    self.ovr2ros_left_hand_twist_sub = rospy.Subscriber("/ovr2ros_left_hand_twist",Twist,self.ovr2ros_left_hand_twist_callback)
    self.ovr2ros_left_hand_inputs_sub = rospy.Subscriber("/ovr2ros_left_hand_inputs",OVR2ROSInputs,self.ovr2ros_left_hand_inputs_callback)
    
    #Puplisher to quest 
    self.ros2ovr_right_hand_haptic_feedback_pub = rospy.Publisher("/ovr2ros_right_hand_haptic_feedback", OVR2ROSHapticFeedback, queue_size=1)
    self.ros2ovr_left_hand_haptic_feedback_pub = rospy.Publisher("/ovr2ros_left_hand_haptic_feedback", OVR2ROSHapticFeedback, queue_size=1)
    self.ros2ovr_dice_twist_pub =rospy.Publisher("/dice_twist", Twist, queue_size=1)
    self.ros2ovr_q2r_twist_pub =rospy.Publisher("/q2r_twist", Twist, queue_size=1)

    #vars
    self.right_hand_pose = PoseStamped()
    self.right_hand_twist = Twist()
    self.right_hand_inputs = OVR2ROSInputs()

    self.left_hand_pose = PoseStamped()
    self.left_hand_twist = Twist()
    self.left_hand_inputs = OVR2ROSInputs()


  def ovr2ros_right_hand_pose_callback(self, data):    
    self.right_hand_pose = data

  def ovr2ros_right_hand_twist_callback(self, data):    
    self.right_hand_twist = data

  def ovr2ros_right_hand_inputs_callback(self, data):    
    self.right_hand_inputs = data

    #if the lower button is pressed send the twist back to the quest to move the q2r ; 0 otherwise
    q2r_twist=Twist()
    if self.right_hand_inputs.button_lower:
      q2r_twist=self.right_hand_twist
    self.ros2ovr_q2r_twist_pub.publish(q2r_twist)

    

    #send the triggers as frequency and amplitude of vibration back to the quest
    right_hand_haptic_feedback = OVR2ROSHapticFeedback()
    right_hand_haptic_feedback.frequency = self.right_hand_inputs.press_index
    right_hand_haptic_feedback.amplitude = self.right_hand_inputs.press_middle
    self.ros2ovr_right_hand_haptic_feedback_pub.publish(right_hand_haptic_feedback)



  def ovr2ros_left_hand_pose_callback(self, data):    
    self.left_hand_pose = data

  def ovr2ros_left_hand_twist_callback(self, data):    
    self.left_hand_twist = data

  def ovr2ros_left_hand_inputs_callback(self, data):    
    self.left_hand_inputs = data


    #if the lower button is pressed send the twist back to the quest to move the dice ; 0 otherwise
    dice_twist=Twist()
    if self.left_hand_inputs.button_lower:
      dice_twist=self.left_hand_twist
    self.ros2ovr_dice_twist_pub.publish(dice_twist)

    

    #send the triggers as frequency and amplitude of vibration back to the quest
    left_hand_haptic_feedback = OVR2ROSHapticFeedback()
    left_hand_haptic_feedback.frequency = self.left_hand_inputs.press_index
    left_hand_haptic_feedback.amplitude = self.left_hand_inputs.press_middle
    self.ros2ovr_left_hand_haptic_feedback_pub.publish(left_hand_haptic_feedback)





def main(args):
  rospy.init_node('quest2rosdemo', anonymous=True)


  r2q = ros2quest()  

  r = rospy.Rate(1000) #1000hz
  #print_commands()
  while not rospy.is_shutdown():
    r.sleep()


if __name__ == '__main__':
    main(sys.argv)