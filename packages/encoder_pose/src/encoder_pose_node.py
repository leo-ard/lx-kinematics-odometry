#!/usr/bin/env python3
import os
import time
from typing import Optional

import numpy as np
import rospy
import yaml
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, EpisodeStart
from nav_msgs.msg import Odometry

from encoder_pose.include.odometry.odometry import delta_phi, estimate_pose

from multiprocessing import Lock

class EncoderPoseNode(DTROS):
    """
    Computes an estimate of the Duckiebot pose using the wheel encoders.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the ROS node
    Configuration:

    Publisher:
        ~encoder_localization (:obj:`PoseStamped`): The computed position
    Subscribers:
        ~/left_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`):
            encoder ticks
        ~/right_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`):
            encoder ticks
    """

    right_tick_prev: Optional[int]
    left_tick_prev: Optional[int]
    delta_phi_left: float
    delta_phi_right: float

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(EncoderPoseNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.log("Initializing...")
        # get the name of the robot
        self.veh = rospy.get_namespace().strip("/")

        self.right_wheel_mutex = Lock()
        self.left_wheel_mutex = Lock()

        # Init the parameters
        self.resetParameters()

        # nominal R and L, you may change these if needed:

        self.R = 0.0318  # meters, default value of wheel radius
        self.baseline = 0.1  # meters, default value of baseline

        # Defining subscribers:

        # Wheel encoder subscriber:
        left_encoder_topic = f"/{self.veh}/left_wheel_encoder_driver_node/tick"
        rospy.Subscriber(left_encoder_topic, WheelEncoderStamped, self.cbLeftEncoder)

        # Wheel encoder subscriber:
        right_encoder_topic = f"/{self.veh}/right_wheel_encoder_driver_node/tick"
        rospy.Subscriber(right_encoder_topic, WheelEncoderStamped, self.cbRightEncoder)

        # Odometry publisher
        self.db_estimated_pose = rospy.Publisher(
            f"/{self.veh}/encoder_localization", Odometry, queue_size=1, dt_topic_type=TopicType.LOCALIZATION
        )



        self.log("Initialized.")
        rospy.Timer(rospy.Duration(1.0/2.0), self.posePublisher)

    def resetParameters(self):
        # Add the node parameters to the parameters dictionary
        self.delta_phi_left = 0.0
        self.left_tick_prev = None

        self.delta_phi_right = 0.0
        self.right_tick_prev = None

        # Initializing the odometry
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.theta_prev = 0.0


    def cbLeftEncoder(self, encoder_msg):
        """
        Wheel encoder callback
        Args:
            encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        with self.left_wheel_mutex:
            # initializing ticks to stored absolute value
            if self.left_tick_prev is None:
                self.left_tick_prev = encoder_msg.data
                return

            # running the DeltaPhi() function copied from the notebooks to calculate rotations
            delta_phi_left, left_ticks_prev = delta_phi(
                encoder_msg.data, self.left_tick_prev, encoder_msg.resolution
            )
            if delta_phi_left == 0:
                return
            self.left_tick_prev = left_ticks_prev
            self.delta_phi_left += delta_phi_left

    def cbRightEncoder(self, encoder_msg):
        """
        Wheel encoder callback, the rotation of the wheel.
        Args:
            encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """

        with self.right_wheel_mutex:
            if self.right_tick_prev is None:
                self.right_tick_prev = encoder_msg.data
                return

            # calculate rotation of right wheel
            delta_phi_right, right_tick_prev = delta_phi(
                encoder_msg.data, self.right_tick_prev, encoder_msg.resolution
            )
            if delta_phi_right == 0:
                return
            self.right_tick_prev = right_tick_prev
            self.delta_phi_right += delta_phi_right

    def posePublisher(self, event=None):
        """
        Publish the pose of the Duckiebot given by the kinematic model
            using the encoders.
        Publish:
            ~/pose (:obj:`PoseStamped`): Duckiebot pose.
        """
        with self.left_wheel_mutex:
            with self.right_wheel_mutex:

                x_curr, y_curr, theta_curr = estimate_pose(
                    self.R,
                    self.baseline,
                    self.x_prev,
                    self.y_prev,
                    self.theta_prev,
                    self.delta_phi_left,
                    self.delta_phi_right,
                )

                if (x_curr == self.x_prev) and (y_curr == self.y_prev) and (theta_curr == self.theta_prev):
                    return

                theta_curr = self.angle_clamp(theta_curr)  # angle always between 0,2pi

                # self.loging to screen for debugging purposes
                self.log("              ODOMETRY             ")
                # self.log(f"Baseline : {self.baseline}   R: {self.R}")
                self.log("Just this move:")
                self.log(f"Theta : {np.rad2deg(theta_curr) - np.rad2deg(self.theta_prev)} deg,  x: {x_curr - self.x_prev} m,  y: {y_curr - self.y_prev} m")
                self.log("Total accumulated:")
                self.log(f"Theta : {np.rad2deg(theta_curr)} deg,  x: {x_curr} m,  y: {y_curr} m")

                self.log(
                    f"Rotation left wheel : {np.rad2deg(self.delta_phi_left)} deg,   "
                    f"Rotation right wheel : {np.rad2deg(self.delta_phi_right)} deg"
                )
                self.log(f"Prev Ticks left : {self.left_tick_prev}   Prev Ticks right : {self.right_tick_prev}")
                # self.log(
                #     f"Prev integral error : {self.prev_int}")

                # Calculate new odometry only when new data from encoders arrives
                self.delta_phi_left = 0
                self.delta_phi_right = 0

                # Current estimate becomes previous estimate at next iteration
                self.x_prev = x_curr
                self.y_prev = y_curr
                self.theta_prev = theta_curr

                # Creating message to plot pose in RVIZ
                odom = Odometry()
                odom.header.frame_id = "map"
                odom.header.stamp = rospy.Time.now()

                odom.pose.pose.position.x = x_curr  # x position - estimate
                odom.pose.pose.position.y = y_curr  # y position - estimate
                odom.pose.pose.position.z = 0  # z position - no flying allowed in Duckietown

                # these are quaternions - stuff for a different course!
                odom.pose.pose.orientation.x = 0
                odom.pose.pose.orientation.y = 0
                odom.pose.pose.orientation.z = np.sin(theta_curr / 2)
                odom.pose.pose.orientation.w = np.cos(theta_curr / 2)

                self.db_estimated_pose.publish(odom)



    def onShutdown(self):
        super(EncoderPoseNode, self).on_shutdown()

    @staticmethod
    def angle_clamp(theta):
        if theta > 2 * np.pi:
            return theta - 2 * np.pi
        elif theta < -2 * np.pi:
            return theta + 2 * np.pi
        else:
            return theta


if __name__ == "__main__":
    # Initialize the node
    encoder_pose_node = EncoderPoseNode(node_name="encoder_pose_node")
    # Keep it spinning
    rospy.spin()
