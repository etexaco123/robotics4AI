from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy
import actionlib
from approach_objects.msg import ApproachObjectResult
from enum import Enum, unique
from rospkg import RosPack
from os.path import join
import rosparam
from std_msgs.msg import String


simulation = False
verbose = True # set true to plot the actions of the robot.

@unique
class NavigationInstruction(Enum):
	searchLocation = 0
	dropLocation = 1
	startLocation = 2


class main_behaviour(AbstractBehaviour):


	def init(self):
		# setpoints
		self.recoverNavigationMaxAttempts = 2
		self.recoverApproachMaxAttempts = 2

		# sub behaviours
		self.behaviour_receive_order = self.get_behaviour("receive_order")
		self.behaviour_nagivate = self.get_behaviour("navigate")
		self.behaviour_approach = self.get_behaviour("approach")
		self.behaviour_recognize = self.get_behaviour("recognize")
		self.behaviour_grasp = self.get_behaviour("grasp")
		self.behaviour_drop = self.get_behaviour("drop")

		# Environment and behaviour settings
		rospack = RosPack()
		path = rospack.get_path("behaviours")

		if simulation:
			data = rosparam.load_file(join(path, "scripts/behaviours/main_behaviour/waypointsSim.yaml"))[0][0]
			self.pickLocations = [data["location1"], data["location2"]]#, data["location3"]]
		else:
			data = rosparam.load_file(join(path, "scripts/behaviours/main_behaviour/waypointsRL.yaml"))[0][0]
			self.pickLocations = [data["location1"], data["location2"]]

		self.recoverLocation = data["recoverLocation"]
		self.startLocation = data["startLocation"]
		self.dropLocation = data["dropLocation"]
		
		self.numberOfPickLocations = len(self.pickLocations)
			
		# variables of program
		self.order = []
		self.droppedObjects = []
		self.currentPickLocation = 0
		self.recoverNavigationAttempt = 0
		self.recoverApproachAttempt = 0
		self.navigationInstruction = NavigationInstruction.startLocation
		self.recoverNavigation = True
		self.firstStartProcedure = True

		# information publisher
		self.information_publisher = rospy.Publisher("/information", String, queue_size = 1)
		
	
	def start(self):
		self.firstStartProcedure = True
		self.recoverNavigation = True
		self.navigationInstruction = NavigationInstruction.startLocation
		self.state=State.navigate
		#self.startProcedure()

	def startProcedure(self):
		# reset variables for the next run
		self.recoverNavigationAttempt = 0
		self.recoverApproachAttempt = 0
		self.currentPickLocation = 0
		self.order = []
		self.droppedObjects = []
		self.recognizedObject = None
		self.firstStartProcedure = False

		# change state to receive an order
		self.state = State.receiveOrder
		

	def update(self):
		
		# Receive order
		if self.state == State.receiveOrder:
			if self.behaviour_receive_order.get_state() == State.idle:
				self.information_publisher.publish("Ready to receive an order")
				self.behaviour_receive_order.start()
			elif self.behaviour_receive_order.get_state() == State.finished:
				self.behaviour_receive_order.setIdle()
				self.order = self.behaviour_receive_order.getResult()
				orderText = str(self.order).replace('[', '').replace(']', '')
				self.information_publisher.publish("Received order: " + orderText)
				self.navigationInstruction = NavigationInstruction.searchLocation
				self.state = State.navigate
			elif self.behaviour_receive_order.get_state() == State.failed:
				self.behaviour_receive_order.setIdle()
				self.information_publisher.publish("Empty order received, ready to receive a new order")

		# Navigation:
		# The navigation procedure has the variable navigationInstruction to specify the navigation action. 
		# If the instruction is searchLocation, it will navigate to search location number currentPickLocation. 
		# With the instructions, the robot can also be navigated to the drop location and start location. 
		# The procedure also has the variable recoverNaviation. If true, the robot will navigate to the starting position. 
		# If navigating with recovering succeeds, it will continue with the still existing navigationInstruction.

		elif self.state == State.navigate:
			if self.behaviour_nagivate.get_state() == State.idle:	
				if self.recoverNavigation:
					navigationLocation = self.recoverLocation
				else:
					if self.navigationInstruction == NavigationInstruction.searchLocation:
						navigationLocation = self.pickLocations[self.currentPickLocation]
					elif self.navigationInstruction == NavigationInstruction.dropLocation:
						navigationLocation = self.dropLocation
					elif self.navigationInstruction == NavigationInstruction.startLocation:
						navigationLocation = self.startLocation

				self.behaviour_nagivate.setGoal(navigationLocation)
				self.behaviour_nagivate.start()
				if verbose: print("Autoprogram: navigation instruction " + str(self.navigationInstruction) + ", recover is "+ str(self.recoverNavigation) + ", location is "+ str(self.currentPickLocation))
			
			#Since the navigation state is used multiple times per run, the next state when navigation succeeds or fails depends on the navigation instruction. 
			elif self.behaviour_nagivate.get_state() == State.finished:
				if verbose: print("Autoprogram: navigation succeeded")
				self.behaviour_nagivate.setIdle()
				
				# if recoverNavigation is true, the state should not be changed.
				if not self.recoverNavigation:
					self.recoverNavigationAttempt = 0
					if self.navigationInstruction == NavigationInstruction.searchLocation:
						self.state = State.approach
					elif self.navigationInstruction == NavigationInstruction.dropLocation:
						self.state = State.drop
					elif self.navigationInstruction == NavigationInstruction.startLocation:
						if not self.firstStartProcedure:
							if len(self.droppedObjects) == 0:
								droppedText = "none"
							else:
								droppedText = str(self.droppedObjects).replace('[', '').replace(']', '')		
							self.information_publisher.publish("Dropped objects: " + droppedText)
						self.startProcedure()
				self.recoverNavigation = False
				
				
			elif self.behaviour_nagivate.get_state() == State.failed:
				self.behaviour_nagivate.setIdle()
				self.recoverNavigation = False
				if self.recoverNavigationAttempt >= self.recoverNavigationMaxAttempts:
					if verbose: print("Autoprogram: naviation failed recovering failed too many times, continue to next search location")
					self.recoverNavigationAttempt = 0
					if self.navigationInstruction == NavigationInstruction.searchLocation:
						self.incrementSearchLocation()
					elif self.navigationInstruction == NavigationInstruction.dropLocation:
						self.fail("failed to recover and move to drop location")
					elif self.navigationInstruction == NavigationInstruction.startLocation:
						self.fail("failed to recover and to return to starting position")
				else:
					if verbose: print("Autoprogram: navigation failed, try to recover")
					self.recoverNavigationAttempt += 1
					self.recoverNavigation = True

		# Approach:
		elif self.state == State.approach:
			if self.behaviour_approach.get_state() == State.idle:
				self.behaviour_approach.start()

			# if it finishes, the state will be recognize. 
			elif self.behaviour_approach.get_state() == State.finished:
				self.behaviour_approach.setIdle()
				self.recoverApproachAttempt = 0
				self.state = State.recognize
				if verbose: print("Autoprogram: approach object has found an object")

			# If it fails, it will try recover navigation to try again if it failed less than "recoverApproachMaxAttempts"
			elif self.behaviour_approach.get_state() == State.failed:
				self.behaviour_approach.setIdle()
				result = self.behaviour_approach.getResult()
				if result.error_code == ApproachObjectResult.NO_OBJECTS_FOUND:
					if verbose: print("Autoprogram: approach failed (no object found). Visit next location")
					self.recoverApproachAttempt = 0
					self.incrementSearchLocation()
				else:				
					if self.recoverApproachAttempt >= self.recoverApproachMaxAttempts:
						if verbose: print("Autoprogram: approach failed (object found). Too many approach attempts, continue to next search location.")
						self.recoverApproachAttempt = 0
						self.incrementSearchLocation()	
					else:
						if verbose: print("Autoprogram: approach failed (object found), try to recover")
						self.recoverApproachAttempt += 1
						self.recoverNavigation = True
						self.state = State.navigate

		# Recognize
		elif self.state == State.recognize:
			if self.behaviour_recognize.get_state() == State.idle:
				self.behaviour_recognize.start()
			elif self.behaviour_recognize.get_state() == State.finished:
				self.behaviour_recognize.setIdle()
				result = self.behaviour_recognize.getResult()

				# print results. If there are no objects found, do not print anything
				for index, objectName in enumerate(result.object_name):
					stringX = "{:.2f}".format(result.xCoordinates[index])
					stringY = "{:.2f}".format(result.yCoordinates[index])
					stringZ = "{:.2f}".format(result.zCoordinates[index])
					print("Object " + objectName + " has coordinate (" + stringX +  ", " + stringY +  ", "+ stringZ +  ")")
					self.recognizedObject = objectName


				#self.recognizedObject = str(result.object_name)
				self.information_publisher.publish("Label name of recognized object: " + self.recognizedObject)
				print("Label name of recognized object: " + self.recognizedObject)

				if self.recognizedObject in self.order:
					self.state = State.grasp
				else:
					self.incrementSearchLocation()
			elif self.behaviour_recognize.get_state() == State.failed:
				self.behaviour_recognize.setIdle()

		# grasp
		if self.state == State.grasp:
			if self.behaviour_grasp.get_state() == State.idle:
				rospy.sleep(1)
				self.information_publisher.publish("Going to grasp object: " + self.recognizedObject)
				print("Going to grasp object: " + self.recognizedObject)
				self.behaviour_grasp.start()
			elif self.behaviour_grasp.get_state() == State.finished:
				self.behaviour_grasp.setIdle()
				self.navigationInstruction = NavigationInstruction.dropLocation
				self.state = State.navigate
			elif self.behaviour_grasp.get_state() == State.failed:
				# the robot can only drive if the arm is in rest position. 
				# this can be checked with: self.behaviour_grasp.getResult()
				saveToDrive = self.behaviour_grasp.getResult()
				self.behaviour_grasp.setIdle()
				self.fail("grasp failed")
		
		# drop
		elif self.state == State.drop:
			if self.behaviour_drop.get_state() == State.idle:
				if verbose: print("Autoprogram: start drop")
				self.behaviour_drop.start()
			elif self.behaviour_drop.get_state() == State.finished:
				self.behaviour_drop.setIdle()
				self.information_publisher.publish("Dropped object: " + self.recognizedObject)
				self.order.remove(self.recognizedObject)
				self.droppedObjects.append(self.recognizedObject)
				if len(self.order) == 0:
					self.navigationInstruction = NavigationInstruction.startLocation
					self.state = State.navigate
				else:
					self.incrementSearchLocation()
					
			elif self.behaviour_drop.get_state() == State.failed:
				if verbose: print("Autoprogram: drop failed")
				saveToDrive = self.behaviour_grasp.getResult()
				self.behaviour_drop.setIdle()
				# the robot can only drive if the arm is in rest position. 
				# this can be checked with: self.behaviour_drop.getResult()
				self.fail("drop failed")



	def incrementSearchLocation(self):
		"""
		When the next search location should be vistited, this function is called.
		It checks if there is a next search location. 
		If there is not, it changes the state to navigation in order to drive the robot back to the start location
		"""
		self.navigationInstruction = NavigationInstruction.searchLocation
		self.state = State.navigate
		if self.currentPickLocation >= self.numberOfPickLocations-1:
			if verbose: print("Autoprogram: all search locations were visited, go back to start location.")
			self.recognizedObject = None
			self.navigationInstruction = NavigationInstruction.startLocation
		else:
			self.currentPickLocation += 1
