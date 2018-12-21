import time

_gpio_states = {}

class FSMLogger():
    def __init__(self):
        self.f = open("fsm.csv", "w")
        self.f.write("time,state\n")
    def __del__(self):
        self.f.close()
    def log(self, state):
        self.f.write("%f,%s\n" % (time.time(), state))

class PWM():
    def __init__(self, ctr, freq):
        global _gpio_states
        if ctr not in _gpio_states:
            raise Exception("Did not call setup on GPIO pin before using PWM")

        self.ctr = ctr
        self.freq = freq
        self.started = False
        self.duty = 0


    def log(self):
        self.logfile.write("%f,%f,%f\n" % (time.time(), self.freq, self.duty))

    def start(self, duty):
        self.logfile = open("pwm_%d.csv" % self.ctr, 'w')
        self.logfile.write("time,freq,duty\n")
        self.duty = duty
        self.started = True
        self.log()

    def stop(self):
        self.duty = 0
        self.log()
        self.logfile.close()
        self.started = False

    def ChangeDutyCycle(self, duty):
        if not self.started:
            raise Exception("Did not call .start() before calling ChangeDutyCycle on PWM")
        self.duty = duty
        self.log()

    def __del__(self):
        if self.duty != 0:
                raise Exception("Forgot to call .stop() on PWM")

_setmode = False
class GPIO():

    BOARD = "BOARD"
    OUT = "OUT"
    IN = "IN"

    @staticmethod
    def setmode(mode):
        global _setmode
        _setmode = True

    @staticmethod
    def setup(ctr, direction, initial):
        global _setmode
        global _gpio_states

        if not _setmode:
            raise Exception("Failed to call setmode before calling setup")

        _gpio_states[ctr] = {'direction': direction, 'initial': initial}

        if initial != 0:
            raise Exception("Initial value of PWM pin not set to zero!")

    @staticmethod
    def cleanup():
        pass

    PWM = PWM
