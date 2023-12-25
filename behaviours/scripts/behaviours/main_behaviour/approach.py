import rospy
from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import actionlib
from approach_objects.msg import ApproachObjectAction, ApproachObjectGoal, ApproachObjectResult

class approach(AbstractBehaviour):
    
    
    def init(self):
        print("connect to approach")
        self.approachClient = actionlib.SimpleActionClient("/approach_objects", ApproachObjectAction)
        self.approachClient.wait_for_server()
        self.state = State.idle
        self.result = None

    def start(self):
        goal = ApproachObjectGoal
        self.approachClient.send_goal(goal)
        self.state = State.running
    
    def update(self):
        if self.state == State.running:
            server_state = self.approachClient.get_state()
            if server_state == actionlib.GoalStatus.SUCCEEDED:
                # if the function succeeded, it does not mean it has found an object.
                # only finish the state is the object has been found. 
                self.result = self.approachClient.get_result()
                if self.result.error_code == ApproachObjectResult.NO_OBJECTS_FOUND:
                    self.state = State.failed
                else:
                    self.state = State.finished          
            elif server_state == actionlib.GoalStatus.ABORTED:
                self.state = State.failed
    
    def getResult(self):
        return self.result

    def setIdle(self):
        self.state = State.idle

    def reset(self):
        self.init()