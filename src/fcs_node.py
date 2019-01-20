#!/usr/bin/env python

import rospy
import pigpio
from tails.msg import FlightControlSurfaces
from std_msgs.msg import Bool, String

class FCS():
    def __init__(self):
        rospy.init_node('fcs')

        self.pi = pigpio.pi()

        #set pins as output and as low
        self.gpio_ail = rospy.get_param('~gpio_ail', 7)
        self.gpio_ele = rospy.get_param('~gpio_ele', 8)
        self.gpio_thr = rospy.get_param('~gpio_thr', 13)
        self.gpio_rud = rospy.get_param('~gpio_rud', 12)

        self.land_ail = rospy.get_param('~land_ail', 5.0 / 100 * 255)
        self.land_ele = rospy.get_param('~land_ele', 5.0 / 100 * 255)
        self.land_thr = rospy.get_param('~land_thr', 5.3 / 100 * 255)
        self.land_rud = rospy.get_param('~land_rud', 5.0 / 100 * 255)

        self.watchdog_timeout = rospy.get_param('~watchdog_timeout', 10.0)

        for gpio in [self.gpio_ail, self.gpio_ele, self.gpio_thr, self.gpio_rud]:
            self.pi.set_mode(gpio, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(gpio, rospy.get_param('pwm_freq', 50))
            self.pi.set_PWM_dutycycle(gpio, 5.0 / 100 * 255)

        self.last_message_time = rospy.get_time() - 5.0

        self.armed = rospy.get_param('~armed', False)
        rospy.loginfo("ARMED: %s", self.armed)

        self.latch_fcs_warn = False
        self.state = "idle"

        rospy.Subscriber('fcs', FlightControlSurfaces,
                         self.ros_fcs_callback, queue_size=100)
        rospy.Subscriber('arm', Bool, self.ros_arm_callback, queue_size=10)
        rospy.Subscriber('state', String, self.ros_state_callback, queue_size=10)

    def ros_state_callback(self, state):
        self.state = state.data

    def ros_arm_callback(self, arm):
        if self.state == "idle":
            rospy.loginfo("ARMED: %s" % arm.data)
            self.armed = arm.data
        else:
            rospy.logerr("Attempted to change ARM while in %s state" % self.state)

    def ros_fcs_callback(self, fcs):
        if self.armed:
            ail = fcs.aileron / 100 * 255
            ele = fcs.elevator / 100 * 255
            thr = fcs.thrust / 100 * 255
            rud = fcs.rudder / 100 * 255

            self.pi.set_PWM_dutycycle(self.gpio_ail, ail)
            self.pi.set_PWM_dutycycle(self.gpio_ele, ele)
            self.pi.set_PWM_dutycycle(self.gpio_thr, thr)
            self.pi.set_PWM_dutycycle(self.gpio_rud, rud)

            self.latch_fcs_warn = False
        else:
            if not self.latch_fcs_warn:
                rospy.logwarn("Recieved FlightControlSurfaces message but not armed.")
            self.latch_fcs_warn = True

        self.last_message_time = rospy.get_time()

    def run(self):
        latch_land = False

        r = rospy.Rate(10)
        while True:

            # Only allow shutdown when disarmed
            if not self.armed and rospy.is_shutdown():
                break

            # Automatically land if we are armed and lose connection to tails node
            if self.armed and rospy.get_time() - self.last_message_time > self.watchdog_timeout:

                self.pi.set_PWM_dutycycle(self.gpio_ail, self.land_ail)
                self.pi.set_PWM_dutycycle(self.gpio_ele, self.land_ele)
                self.pi.set_PWM_dutycycle(self.gpio_thr, self.land_thr)
                self.pi.set_PWM_dutycycle(self.gpio_rud, self.land_rud)

                if not latch_land:
                    rospy.logwarn("Lost connection to tails node, landing.")
                latch_land = True
            else:
                latch_land = False

            r.sleep()

        # TODO: Are all these values correct for shutdown?
        for gpio in [self.gpio_ail, self.gpio_ele, self.gpio_thr, self.gpio_rud]:
            self.pi.set_mode(gpio, pigpio.OUTPUT)
            self.pi.set_PWM_dutycycle(gpio, 5.0 / 100 * 255)

if __name__ == '__main__':
    f = FCS()
    f.run()
