#include "bounding_box_server/bounding_box_server.h"

//! Function to transform a sensor_msgs::PointCloud2 to a different frame of reference.
/**
 * Will wait for transform to become available for 1 seconds, this will mostly fail if the server is just started and a request to transform
 * is giving right away, the transform_listener needs some time to initialize. 
 * It will convert all the points in the Point Cloud to the new frame of reference coordinate system, it will not change the size or dimensions of the 
 * Point Cloud.
 * \param point_cloud_message is the input Point Cloud Message.
 * \param transformed_point_cloud_message is the transformed Point Cloud Message.
 * \param transform_to_link indicates to which link (frame of reference) the Point Cloud Message needs to transform to.
 * \return true if the transform was available and possible, false if either transform was not available or could not transform.
 */
bool BoundingBoxServer::transformPointCloudToLink(PointCloudPtr &source_point_cloud,
                                                  PointCloudPtr &transformed_point_cloud,
                                                  std::string transform_to_link) {
  
  if (buffer_.canTransform(transform_to_link, source_point_cloud->header.frame_id, ros::Time(0))) {
    geometry_msgs::TransformStamped transform_stamped = buffer_.lookupTransform(transform_to_link, 
                                                                                source_point_cloud->header.frame_id,
                                                                                ros::Time(0));
    pcl_ros::transformPointCloud(*source_point_cloud, *transformed_point_cloud, transform_stamped.transform);
    transformed_point_cloud->header.frame_id = transform_to_link;
    return true;
  }

  return false; 
} 