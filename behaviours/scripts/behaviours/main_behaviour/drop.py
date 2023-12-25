from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy
from manipulation_server.msg import DropAction, DropResult
import actionlib


class drop(AbstractBehaviour):
    
    def init(self):
        print("connect to drop")
        self.dropClient = actionlib.SimpleActionClient("/drop", DropAction)
        self.dropClient.wait_for_server()
        self.state = State.idle
        self.saveToDrive = False
 
    def start(self):
        self.dropClient.send_goal("")
        self.state = State.running

    def update(self):
        if self.state == State.running:
            server_state = self.dropClient.get_state()
            #self.saveToDrive = True
            #self.state = State.finished
            #return


            self.saveToDrive = False
            if server_state == actionlib.GoalStatus.SUCCEEDED:
                result = self.dropClient.get_result()
                if result == None:
                    self.state = State.failed
                    return
                if result.armInRestPos:
                    self.saveToDrive = True

                if result.dropSucceeded and result.armInRestPos:
                    self.state = State.finished
                else:
                    self.state = State.failed
            elif server_state == actionlib.GoalStatus.ABORTED:
                self.state = State.failed
    
    def setIdle(self):
        self.state = State.idle

    def getResult(self):
        return self.saveToDrive
        
    def reset(self):
        self.init()