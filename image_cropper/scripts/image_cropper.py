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
data = [] # holds all the image paths and rois 

if len(sys.argv) != 2: 
  print "Provide object name"
  exit()

path = os.path.join(path, "training")

# Create training path 
if not os.path.exists(path):
  os.mkdir(path)

object_name = sys.argv[1]
object_path = os.path.join(path, object_name)

# Create object path 
if not os.path.exists(object_path):
  os.mkdir(object_path)

# Timed callback
def image_callback(event):
  global count, data
  img_msg = rospy.wait_for_message(topic, Image) # Get a image from the camera topic
  image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8") # Convert to OpenCv format
  roi_image = image.copy()

  try:
    result = roi_client(roi_msg)
  except:
    return

  if len(result.object_rois) != 1:
    return
  
  roi = result.object_rois[0]
  cv2.rectangle(roi_image, (roi.left, roi.top), (roi.right, roi.bottom), (255, 0, 0))

  cv2.imshow("image_roi", roi_image)
  key = cv2.waitKey(1) & 0xFF

  if key == ord("q"):

    # Save all data to data.csv 
    with open(os.path.join(object_path, "data.csv"),'wb') as f:
      writer = csv.writer(f)
      writer.writerows(data)
    
    rospy.signal_shutdown("done")
    return
  
  # Append image data, and save full image
  if key == ord(" "):
    save_path = os.path.join(object_path, str(count)+".jpg")
    count += 1
    cv2.imwrite(save_path, image)
    data.append([save_path, roi.top, roi.bottom, roi.left, roi.right])
    print "Images:", count

## init ros and start callback function
rospy.init_node("image_cropper")
cv_bridge = CvBridge()

roi_client = rospy.ServiceProxy("get_object_roi", GetObjectROI)

roi_msg = GetObjectROIRequest()
roi_msg.transform_to_link = "arm_link0"
roi_msg.surface_threshold = 0.015
roi_msg.cluster_distance = 0.05
roi_msg.filter_x = True
roi_msg.min_x = 0.0
roi_msg.max_x = 0.4
roi_msg.filter_y = True
roi_msg.min_y = -0.35
roi_msg.max_y = 0.35
roi_msg.filter_z = False
roi_msg.min_cluster_points = 100
roi_msg.max_cluster_points = 50000
roi_msg.voxelize_cloud = True
roi_msg.leaf_size = 0.005

timer = rospy.Timer(rospy.Duration(1.0/6.0), image_callback)
rospy.spin()
