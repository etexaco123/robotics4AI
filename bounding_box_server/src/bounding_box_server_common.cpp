#include "bounding_box_server/bounding_box_server.h"

//! Function to receive the latest Point Cloud Message.
/**
 * Uses waitForMessage() to receive a PointCloud message on the #point_cloud_topic_ topic, will wait 
 * for 2.0 seconds before returning a nullptr. 
 * \param point_cloud reference to which the newest Point Cloud will be returned to if one can be gathered.
 * \return true when a PointCloud was received, false if no message was received after 2.0 second.
 */
bool BoundingBoxServer::getPointCloud(PointCloudPtr &point_cloud) {
  boost::shared_ptr<const PointCloud> point_cloud_shared_ptr;
  point_cloud_shared_ptr = ros::topic::waitForMessage<PointCloud>(point_cloud_topic_, ros::Duration(2.0));

  if (point_cloud_shared_ptr == nullptr) {
    ROS_WARN_STREAM("Could not receive PointCloud2 message after 2 seconds on topic: " << point_cloud_topic_);
    return false;
  }

  *point_cloud = *point_cloud_shared_ptr;
  return true;
}

//! Transforms a given Point Cloud to the origin, including rotation.
/**
 * Rotates and translates a given Point Cloud based on the centroid_vector (translation), and the yaw (rotation) to the origin (0, 0, 0).
 * \param point_cloud Pointer to the input Point Cloud.
 * \param centered_point_cloud Pointer to the transformed Point Cloud to the origin (0, 0, 0) and -yaw rotation.
 * \param centroid_vector the vector pointing to the center of point_cloud.
 * \param angle the yaw rotation of point_cloud in radians.
 */
void BoundingBoxServer::transformPointCloudToCenter(PointCloudPtr point_cloud, PointCloudPtr centered_point_cloud, Eigen::Vector3f centroid_vector, float angle) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  angle = -angle; // We want to rotate the Point Cloud back to the origin

  // Rotation matrix for the z-axis
  transform(0,0) = std::cos(angle);
  transform(0,1) = -std::sin(angle);
  transform(1,0) = std::sin(angle);
  transform(1,1) = std::cos(angle); 
  // Translation back to origin (0, 0, 0)
  transform.block<3,1>(0,3) = -1.0f * (transform.block<3,3>(0,0) * centroid_vector.head<3>());
  pcl::transformPointCloud(*point_cloud, *centered_point_cloud, transform);
}

