
import rospy
class LowPassFilter(object):
    def __init__(self, tau, ts):
        self.a = 1. / (tau / ts + 1.)
        self.b = tau / ts / (tau / ts + 1.);

        #self.a = 0.8
        #self.b = 0.2;

        self.last_val = 0.
        self.ready = False

    def reset(self):
        #rospy.loginfo("!!!Reset the filter")
        self.last_val = 0.
        self.ready = False

    def get(self):
        return self.last_val

    def filt(self, val):
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val
