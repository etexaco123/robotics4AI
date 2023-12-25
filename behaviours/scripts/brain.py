#!/usr/bin/python
import rospy
from utils.state import State
from utils.AllBehaviours import AllBehaviours

import signal

class Brain(object):

    def __init__(self):
        self.ab = AllBehaviours()
        self.all_behaviours = self.ab.get_all_behaviours()
        
        self.all_behaviours[0].start()
        for b in self.all_behaviours:
            b.init()

        self.rate = rospy.Rate(10)
        self.running = True

    def stop(self):
        self.running = False

    def run(self):

        while self.running:
            # Check if first/main behaviour is not in stop state
            for b in range(len(self.all_behaviours)):
                behaviour = self.all_behaviours[b]
                should_stop = (behaviour.get_state() == State.finished or behaviour.get_state() == State.failed or behaviour.get_state() == State.idle)
                if b == 0 and should_stop:
                    self.running = False

                elif not should_stop:
                    behaviour.update()

            self.rate.sleep()

if __name__ == "__main__":

    rospy.init_node("Brain")
    brain = Brain()

    signal.signal(signal.SIGTERM, brain.stop)
    signal.signal(signal.SIGINT, brain.stop)

    brain.run()
