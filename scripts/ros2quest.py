#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from quest2ros.msg import OVR2ROSInputs, OVR2ROSHapticFeedback
from geometry_msgs.msg import PoseStamped, Twist


class Quest2ROSNode(Node):

    def __init__(self):
        super().__init__('quest2rosdemo')

        # Subscribers from Quest
        self.create_subscription(PoseStamped,   'q2r_right_hand_pose',   self.right_hand_pose_cb,   10)
        self.create_subscription(Twist,         'q2r_right_hand_twist',  self.right_hand_twist_cb,  10)
        self.create_subscription(OVR2ROSInputs, 'q2r_right_hand_inputs', self.right_hand_inputs_cb, 10)
        self.create_subscription(PoseStamped,   'q2r_left_hand_pose',    self.left_hand_pose_cb,    10)
        self.create_subscription(Twist,         'q2r_left_hand_twist',   self.left_hand_twist_cb,   10)
        self.create_subscription(OVR2ROSInputs, 'q2r_left_hand_inputs',  self.left_hand_inputs_cb,  10)

        # Publishers to Quest
        self.right_haptic_pub = self.create_publisher(OVR2ROSHapticFeedback, 'q2r_right_hand_haptic_feedback', 1)
        self.left_haptic_pub  = self.create_publisher(OVR2ROSHapticFeedback, 'q2r_left_hand_haptic_feedback',  1)
        self.dice_twist_pub   = self.create_publisher(Twist, 'dice_twist', 1)
        self.q2r_twist_pub    = self.create_publisher(Twist, 'q2r_twist',  1)

        # State
        self.right_hand_pose   = PoseStamped()
        self.right_hand_twist  = Twist()
        self.right_hand_inputs = OVR2ROSInputs()

        self.left_hand_pose   = PoseStamped()
        self.left_hand_twist  = Twist()
        self.left_hand_inputs = OVR2ROSInputs()

    def right_hand_pose_cb(self, msg):
        self.right_hand_pose = msg

    def right_hand_twist_cb(self, msg):
        self.right_hand_twist = msg

    def right_hand_inputs_cb(self, msg):
        self.right_hand_inputs = msg

        # If lower button pressed, forward twist to move q2r; zero otherwise
        q2r_twist = self.right_hand_twist if msg.button_lower else Twist()
        self.q2r_twist_pub.publish(q2r_twist)

        # Mirror trigger values as haptic feedback
        haptic = OVR2ROSHapticFeedback()
        haptic.frequency = msg.press_index
        haptic.amplitude = msg.press_middle
        self.right_haptic_pub.publish(haptic)

    def left_hand_pose_cb(self, msg):
        self.left_hand_pose = msg

    def left_hand_twist_cb(self, msg):
        self.left_hand_twist = msg

    def left_hand_inputs_cb(self, msg):
        self.left_hand_inputs = msg

        # If lower button pressed, forward twist to move dice; zero otherwise
        dice_twist = self.left_hand_twist if msg.button_lower else Twist()
        self.dice_twist_pub.publish(dice_twist)

        # Mirror trigger values as haptic feedback
        haptic = OVR2ROSHapticFeedback()
        haptic.frequency = msg.press_index
        haptic.amplitude = msg.press_middle
        self.left_haptic_pub.publish(haptic)


def main(args=None):
    rclpy.init(args=args)
    node = Quest2ROSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
