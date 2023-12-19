#!/usr/bin/env python3
#Author: lnotspotl

import rospy

from sensor_msgs.msg import Joy,Imu
from RobotController import RobotController
from InverseKinematics import robot_IK
from std_msgs.msg import Float64

USE_IMU = True
RATE = 60

rospy.init_node("Robot_Controller")

# Robot geometry
body = [0.559, 0.12]
legs = [0.,0.1425, 0.426, 0.345] 

spot_robot = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics = robot_IK.InverseKinematics(body, legs)

command_topics = ["/spot_gazebo/front_right_leg_joint/command",
                  "/spot_gazebo/front_right_thigh_joint/command",
                  "/spot_gazebo/front_right_shin_joint/command",
                  "/spot_gazebo/front_left_leg_joint/command",
                  "/spot_gazebo/front_left_thigh_joint/command",
                  "/spot_gazebo/front_left_shin_joint/command",
                  "/spot_gazebo/rear_right_leg_joint/command",
                  "/spot_gazebo/rear_right_thigh_joint/command",
                  "/spot_gazebo/rear_right_shin_joint/command",
                  "/spot_gazebo/rear_left_leg_joint/command",
                  "/spot_gazebo/rear_left_thigh_joint/command",
                  "/spot_gazebo/rear_left_shin_joint/command"]

publishers = []
for i in range(len(command_topics)):
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 0))

if USE_IMU:
    rospy.Subscriber("spot_imu/base_link_orientation",Imu,spot_robot.imu_orientation)
rospy.Subscriber("spot_joy/joy_ramped",Joy,spot_robot.joystick_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
del RATE

while not rospy.is_shutdown():
    leg_positions = spot_robot.run()
    spot_robot.change_controller()

    dx = spot_robot.state.body_local_position[0]
    dy = spot_robot.state.body_local_position[1]
    dz = spot_robot.state.body_local_position[2]
    
    roll = spot_robot.state.body_local_orientation[0]
    pitch = spot_robot.state.body_local_orientation[1]
    yaw = spot_robot.state.body_local_orientation[2]

    try:
        joint_angles = inverseKinematics.inverse_kinematics(leg_positions,
                               dx, dy, dz, roll, pitch, yaw)

        for i in range(len(joint_angles)):
            publishers[i].publish(joint_angles[i])
    except:
        rospy.loginfo(f"Can not solve inverese kinematics")

    rate.sleep()
