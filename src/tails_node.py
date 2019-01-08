#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import Queue
from collections import deque
from time import time
from transitions import Machine
from transitions import State
import transitions
import logging
import numpy as np


class Tails():
    def __init__(self, update_rate_hz=10, watchdog_timeout=10):
        rospy.init_node('tails')
        rospy.on_shutdown(self.ros_shutdown_signal)

        if rospy.get_param('~simulation_mode', True):
            from mock_tails import GPIO
            from mock_tails import FSMLogger
            self.logger = FSMLogger()
            self.simulation_mode = True
        else:
            import RPi.GPIO as GPIO
            self.simulation_mode = False

        # Threading / Synchronization Primitives
        self.command_queue = Queue.Queue()

        self.update_rate = 1.0 / update_rate_hz
        self.watchdog_timeout = watchdog_timeout
        self.shutdown_flag = False

        self.last_event_time = time()

        self.last_cycle_time = time()

        self.last_imu_linear = deque(maxlen=10)
        self.last_imu_angular = deque(maxlen=10)

        self.state_t = 0.

        self.linear_z_mean = None
        self.linear_z_std = None

        states = ['idle',
                  'launch',
                  'hover',
                  'land',
                  'shutdown',
                  'navigate']

        self.machine = Machine(model=self, states=states, initial='idle')

        self.machine.on_enter_idle('enter_idle')
        self.machine.on_exit_idle('exit_idle')

        self.machine.on_enter_launch('enter_launch')
        self.machine.on_exit_launch('exit_launch')

        self.machine.on_enter_hover('enter_hover')
        self.machine.on_exit_hover('exit_hover')

        self.machine.on_enter_navigate('enter_navigate')
        self.machine.on_exit_navigate('exit_navigate')

        self.machine.on_enter_land('enter_land')
        self.machine.on_exit_land('exit_land')

        self.machine.on_enter_shutdown('enter_shutdown')

        self.machine.add_transition('shutdown', 'idle', 'shutdown')
        self.machine.add_transition('launch', 'idle', 'launch')
        self.machine.add_transition('hover', 'launch', 'hover')
        self.machine.add_transition('start_navigate', 'hover', 'navigate')
        self.machine.add_transition('stop_navigate', 'navigate', 'hover')
        self.machine.add_transition('land', 'hover', 'land')
        self.machine.add_transition('grounded', 'land', 'idle')

        self.shutdown_map = {'idle': 'shutdown',
                                'launch': 'hover',
                                'hover': 'land',
                                'land': 'idle',
                                'navigate': 'stop_navigate'}

        GPIO.setmode(GPIO.BOARD) #change to BOARD numbering schema

        #set pins as output and as low
        ail = 7
        ele = 8
        thr = 11
        rud = 12

        self.ctr = (ail, ele, thr, rud)

        #setup ctr as output channels and set to 1 (ON)
        GPIO.setup(self.ctr[0], GPIO.OUT, initial = 0)
        GPIO.setup(self.ctr[1], GPIO.OUT, initial = 0)
        GPIO.setup(self.ctr[2], GPIO.OUT, initial = 0)
        GPIO.setup(self.ctr[3], GPIO.OUT, initial = 0)

        #start pwm with 100 freq
        self.pwm_ail = GPIO.PWM(self.ctr[0], 50)
        self.pwm_ele = GPIO.PWM(self.ctr[1], 50)
        self.pwm_thr = GPIO.PWM(self.ctr[2], 50)
        self.pwm_rud = GPIO.PWM(self.ctr[3], 50)

        self.pwms = [self.pwm_ail, self.pwm_ele, self.pwm_thr, self.pwm_rud]

        #All in off state
        for pwm in self.pwms:
            pwm.start(5)

        rospy.Subscriber('/cmd', String,
                         self.ros_cmd_callback, queue_size=100)

        rospy.Subscriber('/imu/data_raw', Imu, self.ros_imu_callback, queue_size=10)
    def run(self):

        self.enter_idle()

        while True:
            try:
                command = self.command_queue.get(timeout=self.update_rate).data
            except Queue.Empty:
                command = None

            # We got a new command, reset watchdog timeout
            if command is not None:
                rospy.loginfo("Recieved Command: %s" % command)
                self.last_event_time = time()

            if  self.shutdown_flag or time() - self.last_event_time > self.watchdog_timeout:
                command = self.shutdown_map[self.state]

            if command is not None:
                try:
                    # Attempt to execute a transition if we have one
                    if command == 'shutdown':
                        self.shutdown()
                    elif command == 'launch':
                        self.launch()
                    elif command == 'start_navigate':
                        self.start_navigate()
                    elif command == 'stop_navigate':
                        self.stop_navigate()
                    elif command == 'land':
                        self.land()
                    elif command == 'watchdog':
                        # This command only exists to keep the drone from advancing towards shutdown
                        pass
                except transitions.core.MachineError as e:
                    rospy.logwarn(str(e))

            # Execute the required states function update_rate_hz times a second
            if self.state == 'shutdown':
                return
            elif self.state == 'idle':
                self.control_idle()
            elif self.state == 'launch':
                self.control_launch()
            elif self.state == 'hover':
                self.control_hover()
            elif self.state == 'navigate':
                self.control_navigate()
            elif self.state == 'land':
                self.control_land()
            elif self.state == 'GROUNDED':
                self.control_grounded()

            self.state_t += self.update_rate

    # Drone Control
    def control_idle(self):
        # TODO: Replace me with real logic!
        pass

    def control_launch(self):
        # Calls self.hover() when done
        #increase duty cycle from 0 to 100%
        if self.state_t >= 5:
            self.hover()

    def control_hover(self):
        pass

    def control_navigate(self):
        # Calls self.stop_navigate() when done
        # To go forward we need elevate to "go down" - throttle up
        # 7.5 to 7.6 will slowly rotate the drone right
        if self.state_t >= 15:
            self.stop_navigate()

    def control_land(self):
        # TODO: Make this way more rigorous and use rosbag to record actual landing
        if (self.linear_z_mean >= 1326.0 - 10.0 and self.linear_z_mean <= 1326.0 + 10.0) and self.linear_z_std < 20.0:
            self.grounded()

    # FSM Transitions - Implement as needed
    def enter_idle(self):
        rospy.loginfo("FSM: enter_idle")
        #Intial pwm states
        self.pwm_thr.ChangeDutyCycle(5)
        #5% left, 10% right, 7.5% off
        self.pwm_rud.ChangeDutyCycle(7.5)
        self.pwm_ele.ChangeDutyCycle(7.5)
        self.pwm_ail.ChangeDutyCycle(7.5)

        self.state_t = 0

    def exit_idle(self):
        rospy.loginfo("FSM: exit_idle")
        #does not need implementation

    def enter_launch(self):
        if self.simulation_mode:
            self.logger.log("launch")
        rospy.loginfo("FSM: enter_launch")

        # TODO: find optimal launch duty cycle
        self.pwm_thr.ChangeDutyCycle(5.6)

        self.state_t = 0

    def exit_launch(self):
        rospy.loginfo("FSM: exit_launch")

    def enter_hover(self):
        if self.simulation_mode:
            self.logger.log("hover")
        rospy.loginfo("FSM: enter_hover")

        # TODO: Figure out the duty cycle for hovering
        self.pwm_thr.ChangeDutyCycle(5.45)

        self.state_t = 0

    def exit_hover(self):
        rospy.loginfo("FSM: exit_hover")

    def enter_land(self):
        if self.simulation_mode:
            self.logger.log("land")
        rospy.loginfo("FSM: enter_land")

        # TODO: Figure out the duty cycle which slowly loses altitude
        self.pwm_thr.ChangeDutyCycle(5.3)

        self.state_t = 0

    def exit_land(self):
        rospy.loginfo("FSM: exit_land")

    def enter_navigate(self):
        if self.simulation_mode:
            self.logger.log("navigate")
        rospy.loginfo("FSM: enter_navigate")

        self.state_t = 0

    def exit_navigate(self):
        rospy.loginfo("FSM: exit_navigate")


    def enter_shutdown(self):
        if self.simulation_mode:
            self.logger.log("shutdown")
        rospy.loginfo("FSM: enter_shutdown")
        for pwm in self.pwms:
            pwm.stop()
        if not rospy.get_param('~simulation_mode', True):
            GPIO.cleanup()

        self.state_t = 0

    def ros_cmd_callback(self, cmd):
        self.command_queue.put(cmd)


    def ros_imu_callback(self, imu):
        self.last_imu_linear.append([imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z])
        self.last_imu_angular.append([imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z])

        angular = np.array(self.last_imu_angular)
        linear = np.array(self.last_imu_linear)

        self.linear_z_mean = np.mean(linear[:, 2])
        self.linear_z_std = np.std(linear[:, 2])

        print("u: %f std: %f" % (self.linear_z_mean, self.linear_z_std))

    # ROS Signals
    def ros_shutdown_signal(self):
        self.shutdown_flag = True


if __name__ == '__main__':

    tails = Tails()
    tails.run()

    """
    try:
        tails.run()
    except Exception as e:
        rospy.loginfo("--FATAL EXCEPTION--")
        rospy.loginfo(str(e))
        rospy.loginfo("ATTEMPTING EMERGENCY landING")

        # TODO: Emergency landing
    """
