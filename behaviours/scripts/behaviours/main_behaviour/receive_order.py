import rospy 
from order.msg import Order
from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy

class receive_order(AbstractBehaviour):
    
    
    def init(self):
        self.result = None
        self.state = State.idle

    def start(self):
        self.state = State.running
           

    def update(self):
        try:
            order_msg = rospy.wait_for_message("/order", Order, 0.5) # Wait for 0.5 seconds to receive something
            if order_msg.objects == None:
                self.state = State.failed
            else: 
                self.result = order_msg.objects
                self.state = State.finished 
        except:
            pass

    def getResult(self):
        return self.result

    def setIdle(self):
        self.state = State.idle

    def reset(self):
        self.init()