import rospy
from std_msgs.msg import String
import Queue
from time import time
from transitions.extensions import GraphMachine as Machine
from transitions import State
import transitions
import logging


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

            try:
                if command is not None:
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
        # TODO: Replace me with real logic!
        self.hover()

    def control_hover(self):
        # TODO: Replace me with real logic!
        pass

    def control_navigate(self):
        # Calls self.stop_navigate() when done
        # TODO: Replace me with real logic!
        self.stop_navigate()

    def control_land(self):
        # Calls self.grounded() when done
        # TODO: Replace me with real logic!
        self.grounded()

    # FSM Transitions - Implement as needed
    def enter_idle(self):
        rospy.loginfo("FSM: enter_idle")

    def exit_idle(self):
        rospy.loginfo("FSM: exit_idle")

    def enter_launch(self):
        rospy.loginfo("FSM: enter_launch")

    def exit_launch(self):
        rospy.loginfo("FSM: exit_launch")

    def enter_hover(self):
        rospy.loginfo("FSM: enter_hover")

    def exit_hover(self):
        rospy.loginfo("FSM: exit_hover")

    def enter_land(self):
        rospy.loginfo("FSM: enter_land")

    def exit_land(self):
        rospy.loginfo("FSM: exit_land")

    def enter_navigate(self):
        rospy.loginfo("FSM: enter_navigate")

    def exit_navigate(self):
        rospy.loginfo("FSM: exit_navigate")

    def enter_shutdown(self):
        rospy.loginfo("FSM: enter_shutdown")

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
