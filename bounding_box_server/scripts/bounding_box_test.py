#! /usr/bin/python
import rospy 
from bounding_box_server.srv import GetBoundingBox, GetBoundingBoxRequest

rospy.init_node("testing_bounding_box_service")
bounding_box_client = rospy.ServiceProxy("/get_bounding_boxes", GetBoundingBox)

print "Waiting for get_bounding_box server"
bounding_box_client.wait_for_service()
print "Connected"

goal_msg = GetBoundingBoxRequest()
goal_msg.transform_to = "arm_link0"
goal_msg.cluster_threshold = 0.05 # Set the cluster threshold
goal_msg.surface_threshold = 0.015 # Set the surface threshold
goal_msg.min_x = 0.05 # Set the min x value 
goal_msg.max_x = 0.4 # Max distance to look a from the arm_link0

result = bounding_box_client(goal_msg)
print result
