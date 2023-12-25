import rospy
from sensor_msgs.msg import Image
import cv2
import os
import sys 
import csv
from point_cloud_functions.srv import GetObjectROI, GetObjectROIRequest
from cv_bridge import CvBridge
import numpy as np
import time 

### Params
topic = "/camera/color/image_raw" 
path = os.path.join(os.environ["HOME"], "data")

# Create path
if not os.path.exists(path):
  os.mkdir(path)

count = 0 # Used for giving image unique name
path = os.path.join(path, "robotImages")
lastImage = None

# Create training path 
if not os.path.exists(path):
  os.mkdir(path)

# Timed callback
def image_callback(event):
  global count, lastImage
  save_path = os.path.join(path, str(count)+".jpg")
  img_msg = rospy.wait_for_message(topic, Image) # Get a image from the camera topic
  image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8") # Convert to OpenCv format
  #if lastImage == None:
  #  lastImage = image.copy()
  #  count += 1
  #  cv2.imwrite(save_path, image)
  #  print ("Images:", count)
  #else:
  comparison = lastImage == image
  if not comparison.all():
    lastImage = image.copy()
    count += 1
    cv2.imwrite(save_path, image)
    print ("Images:", count)




  # Save full image
  #save_path = os.path.join(path, str(count)+".jpg")
  #count += 1
  #cv2.imwrite(save_path, image)
  #print "Images:", count

## init ros and start callback function
rospy.init_node("image_cropper")
cv_bridge = CvBridge()

timer = rospy.Timer(rospy.Duration(1.0/6.0), image_callback)
rospy.spin()
