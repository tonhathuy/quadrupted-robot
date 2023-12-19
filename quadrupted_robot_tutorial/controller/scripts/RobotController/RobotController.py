#!/usr/bin/evn python3

import numpy as np
import tf
import rospy

from . StateCommand import State, Command, BehaviorState
from . TrotGaitController import TrotGaitController
from . CrawlGaitController import CrawlGaitController
from . StandController import StandController

class Robot(object):
    def __init__(self, body, legs, imu):
        self.body = body
        self.legs = legs

        self.delta_x = self.body[0] * 0.5
        self.delta_y = self.body[1] * 0.5 + self.legs[1]
        self.default_height = 0.72 
        self.x_shift_front = -0.1 
        self.x_shift_back = -0.02     

        self.trotGaitController = TrotGaitController(self.default_stance,
            stance_time = 0.1, swing_time = 0.24, time_step = 0.02, use_imu = imu) 
        self.crawlGaitController = CrawlGaitController(self.default_stance,
            stance_time = 0.55, swing_time = 0.45, time_step = 0.02)
        self.standController = StandController(self.default_stance)

        self.currentController = self.standController
        self.state = State(self.default_height)
        self.state.foot_locations = self.default_stance
        self.command = Command(self.default_height)

    def change_controller(self):
        
        if self.command.trot_event:
            if self.state.behavior_state == BehaviorState.STAND:
                self.state.behavior_state = BehaviorState.TROT
                self.currentController = self.trotGaitController
                self.currentController.pid_controller.reset()
                self.state.ticks = 0
            self.command.trot_event = False

        elif self.command.crawl_event:
            if self.state.behavior_state == BehaviorState.STAND:
                self.state.behavior_state = BehaviorState.CRAWL
                self.currentController = self.crawlGaitController
                self.currentController.first_cycle = True
                self.state.ticks = 0
            self.command.crawl_event = False

        elif self.command.stand_event:
            self.state.behavior_state = BehaviorState.STAND
            self.currentController = self.standController
            self.currentController.pid_controller.reset()
            self.command.rest_event = False

    def joystick_command(self,msg):
        if msg.buttons[1]: # trot
            rospy.loginfo(f"Trot mode!")
            self.command.trot_event = True
            self.command.crawl_event = False
            self.command.stand_event = False

        elif msg.buttons[2]: # crawl
            rospy.loginfo(f"Crawl mode!")
            self.command.trot_event = False
            self.command.crawl_event = True
            self.command.stand_event = False
       
        elif msg.buttons[0]: # stand
            rospy.loginfo(f"Stand mode!")
            self.command.trot_event = False
            self.command.crawl_event = False
            self.command.stand_event = True

        elif msg.buttons[6]:
            rospy.loginfo(f"Stand up!")
            self.default_height = 0.52 
            self.state.robot_height = -self.default_height
            self.command.robot_height = -self.default_height

        self.currentController.updateStateCommand(msg, self.state, self.command)

    def imu_orientation(self,msg):
        q = msg.orientation
        rpy_angles = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.state.imu_roll = rpy_angles[0]
        self.state.imu_pitch = rpy_angles[1]

    def run(self):
        return self.currentController.run(self.state, self.command)

    @property
    def default_stance(self):
        # FR, FL, RR, RL
        return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    , self.delta_y                    ],
                         [0                                ,0                                ,0                                ,0                                ]])
