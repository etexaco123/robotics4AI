#include "bounding_box_server/bounding_box_server.h"

//! Constructor.
/** Get the point_cloud_topic giving via a launch files, otherwise uses default "/camera/depth_registered/points".
 * Creates the service to get the bounding_boxes
 * \param nh a ros::NodeHandle created in the main function.  
 */ 
BoundingBoxServer::BoundingBoxServer(ros::NodeHandle &nh) :
    nh_(nh),
    bounding_box_publisher_(nh_), 
    transform_listener_(buffer_) {

  ros::param::param(std::string("~point_cloud_topic"), point_cloud_topic_, std::string("/camera/depth_registered/points"));
  get_bounding_boxes_service_ = nh_.advertiseService("/get_bounding_boxes", &BoundingBoxServer::getBoundingBoxesCallback, this);
}
