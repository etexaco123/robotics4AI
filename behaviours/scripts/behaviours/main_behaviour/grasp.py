from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy
from manipulation_server.msg import GraspAction, GraspResult
import actionlib

class grasp(AbstractBehaviour):
    
    
    def init(self):
        print("connect to grasp")
        self.graspClient = actionlib.SimpleActionClient("/grasp", GraspAction)
        self.graspClient.wait_for_server()
        self.state = State.idle
        self.saveToDrive = False

    def start(self):
        self.graspClient.send_goal("")
        self.state = State.running

    def update(self):
        if self.state == State.running:
            server_state = self.graspClient.get_state()
            #self.saveToDrive = True
            #self.state = State.finished
            #return

            self.saveToDrive = False
            if server_state == actionlib.GoalStatus.SUCCEEDED:
                result = self.graspClient.get_result()
                if result == None:
                    self.state = State.failed
                    return
                # if the grasp server already failed at making the bounding box, 
                # or server managed to get the arm in rest position, the robot is save to drive
                if not result.boundingBoxSucceeded or result.armInRestPos:
                    self.saveToDrive = True

                if result.boundingBoxSucceeded and result.graspSucceeded and result.armInRestPos:
                    self.state = State.finished
                else:
                    self.state = State.failed
    
            elif server_state == actionlib.GoalStatus.ABORTED:
                self.state = State.failed

    def getResult(self):
        return self.saveToDrive

    def setIdle(self):
        self.goalParameters = None
        self.state = State.idle

    def reset(self):
        self.init()
