import rospy
from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import actionlib
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class navigate(AbstractBehaviour):
    
    def init(self):
        print("connect to navigate ...")
        self.navigateClient = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.navigateClient.wait_for_server()
        print("done")
        self.goalParameters = None
        self.state = State.idle
        self.idx = 0

    def start(self):
        if self.goalParameters == None:
            print("Exception, no goal loaded")
            return

        targetPosition = self.goalParameters

        #idx = 0
        msg = MoveBaseGoal()
        msg.target_pose.header.seq = self.idx
        self.idx += 1
        msg.target_pose.header.stamp = rospy.Time.now()
        msg.target_pose.header.frame_id = "map"
        msg.target_pose.pose.position.x = targetPosition[0]
        msg.target_pose.pose.position.y = targetPosition[1]
        msg.target_pose.pose.position.z = 0.0
        msg.target_pose.pose.orientation.z = targetPosition[2]
        msg.target_pose.pose.orientation.w = targetPosition[3]

        self.navigateClient.send_goal(msg)     
        self.state = State.running

    def setGoal(self, goalParameters):
        # the goal parameters are the values of the coordinate the robot should navigate to. 
        self.goalParameters = goalParameters
    
    def update(self):
        if self.state == State.running:
            server_state = self.navigateClient.get_state()
            if server_state == actionlib.GoalStatus.SUCCEEDED:
                self.state = State.finished
            elif server_state == actionlib.GoalStatus.ABORTED:
                self.state = State.failed

    def setIdle(self):
        self.state = State.idle

    def reset(self):
        self.init()