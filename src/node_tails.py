import rospy
from std_msgs.msg import String
import Queue
from time import time
from transitions.extensions import GraphMachine as Machine
from transitions import State
import transitions
import logging
import Rpi.GPIO as GPIO
import time

# For increasing pwm signal
def frange(x, y, jump):
  while x < y:
    yield x
    x += jump

# For decreasing pwm signal 
def frange_r(x, y, jump):
  while y < x:
    yield x
    x -= jump    

GPIO.setmode(GPIO.BOARD) #change to BOARD numbering schema

#set pins as output and as low
ail = 7
ele = 8
thr = 11
rud = 12

ctr = (ail, ele, thr, rud)

#setup ctr as output channels and set to 1 (ON)
GPIO.setup(ctr, GPIO.OUT, initial = 1)

#start pwm with 100 freq
pwm_ail = GPIO.PWM(ctr[0], 50)
pwm_ele = GPIO.PWM(ctr[1], 50)
pwm_thr = GPIO.PWM(ctr[2], 50)
pwm_rud = GPIO.PWM(ctr[3], 50)

#All in off state
pwm_ail.start(0)
pwm_ele.start(0)
pwm_thr.start(0)
pwm_rud.start(0)

#Intial pwm states
pwm_rud.ChangeDutyCycle(10)
pwm_thr.ChangeDutyCycle(5)
time.sleep(2)
#5% left, 10% right, 7.5% off
pwm_rud.ChangeDutyCycle(7.5)
pwm_ele.ChangeDutyCycle(7.5)
pwm_ail.ChangeDutyCycle(7.5)

class Tails():
    def __init__(self, update_rate_hz=10, watchdog_timeout=10):
        rospy.init_node('tails')
        rospy.on_shutdown(self.ros_shutdown_signal)

        # Threading / Synchronization Primitives
        self.command_queue = Queue.Queue()

        self.update_rate = 1/update_rate_hz
        self.watchdog_timeout = watchdog_timeout
        self.shutdown_flag = False

        self.last_event_time = time()

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

        try:
            self.machine.get_graph().draw('visualize_finite_state_machine.png', prog='dot')
        except:
            pass

        self.shutdown_map = {'idle': 'shutdown',
                                'launch': 'hover',
                                'hover': 'land',
                                'land': 'idle',
                                'navigate': 'stop_navigate'}

        rospy.Subscriber('/cmd', String,
                         self.ros_cmd_callback, queue_size=100)
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

    # Drone Control
    def control_idle(self):
        # TODO: Replace me with real logic!
        pass

    def control_launch(self):
        # Calls self.hover() when done
        #increase duty cycle from 0 to 100%
        
        self.hover()

    def control_hover(self):
        # Will remain hovering indefinetly until
        # no input is recieved.  
        pass

    def control_navigate(self):
        # Calls self.stop_navigate() when done
        # To go forward we need elevate to "go down" - throttle up
        # 7.5 to 7.6 will slowly rotate the drone right
        pwm_rud.ChangeDutyCycle(8)
        self.stop_navigate()

    def control_land(self):
        # Calls self.grounded() when done
        
        self.grounded()

    # FSM Transitions - Implement as needed
    def enter_idle(self):
        rospy.loginfo("FSM: enter_idle")
        #Intial pwm states
        pwm_rud.ChangeDutyCycle(10)
        pwm_thr.ChangeDutyCycle(5)
        time.sleep(2)
        #5% left, 10% right, 7.5% off
        pwm_rud.ChangeDutyCycle(7.5)
        pwm_ele.ChangeDutyCycle(7.5)
        pwm_ail.ChangeDutyCycle(7.5)

    def exit_idle(self):
        rospy.loginfo("FSM: exit_idle")
        #does not need implementation

    def enter_launch(self):
        rospy.loginfo("FSM: enter_launch")
        for dc in frange(5.0, 5.5, 0.1):
            pwm_thr.ChangeDutyCycle(dc)
        time.sleep(0.1)

    def exit_launch(self):
        rospy.loginfo("FSM: exit_launch") 

    def enter_hover(self):
        rospy.loginfo("FSM: enter_hover")

    def exit_hover(self):
        rospy.loginfo("FSM: exit_hover")

    def enter_land(self):
        rospy.loginfo("FSM: enter_land")
        for dc in frange_r(5.5, 5.0, 0.1):
            pwm_thr.ChangeDutyCycle(dc)
        time.sleep(0.1)

    def exit_land(self):
        rospy.loginfo("FSM: exit_land")

    def enter_navigate(self):
        rospy.loginfo("FSM: enter_navigate")
        for dc in frange(7.5, 8, 0.5):
            pwm_rud.ChangeDutyCycle(dc)
            #pwm_ele.ChangeDutyCycle(dc)

    def exit_navigate(self):
        rospy.loginfo("FSM: exit_navigate")
        for dc in frange_r(8, 7.5, 0.5):
            pwm_rud.ChangeDutyCycle(dc)

    def enter_shutdown(self):
        rospy.loginfo("FSM: enter_shutdown")
        pwm.stop()
        GPIO.cleanup()

    def ros_cmd_callback(self, cmd):
        self.command_queue.put(cmd)

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
