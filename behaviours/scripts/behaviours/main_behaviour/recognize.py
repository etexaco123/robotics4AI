from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy
from object_recognition.msg import ObjectRecognitionGoal, ObjectRecognitionAction, ObjectRecognitionResult
import actionlib

class recognize(AbstractBehaviour):
    
    
    def init(self):
        print("connect to recognize")
        self.recognizeClient = actionlib.SimpleActionClient("/object_recognition", ObjectRecognitionAction)
        self.recognizeClient.wait_for_server()
        self.result = None
    
    def start(self):
        self.result = None
        self.recognizeClient.send_goal(ObjectRecognitionGoal())
        self.state = State.running

    def update(self):
        if self.state == State.running:
            server_state = self.recognizeClient.get_state()
            if server_state == actionlib.GoalStatus.SUCCEEDED:
                self.result = self.recognizeClient.get_result()
                if self.result == None:
                    self.state = State.failed
                elif len(self.result.object_name) == 0:
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