#include "bounding_box_server/bounding_box_server.h"

//! Removes a flat surface from the Point Cloud.
/**
 * Removes 1 flat surface, which should be the floor, from the Point Cloud. It uses RANSAC to find points belonging to a possible 
 * planar surface, and checks if the points lay within a threshold to determine if it belongs to the surface or not. 
 * \param point_cloud Pointer to the input Point Cloud, should only contain one large planar surface (the floor).
 * \param floorless_point_cloud Pointer to the Point Cloud without a floor surface in it.
 * \param distance_threshold determines the threshold in meters when a points belongs to a possible planar surface or not.
 */
void BoundingBoxServer::removeFloor(PointCloudPtr point_cloud, PointCloudPtr floorless_point_cloud, float distance_threshold) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); 
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> segmenter;
  segmenter.setInputCloud (point_cloud);
  segmenter.setDistanceThreshold (distance_threshold);
  segmenter.setMethodType (pcl::SAC_RANSAC);
  segmenter.setModelType (pcl::SACMODEL_PLANE);
  segmenter.segment (*inliers, *coefficients);
  
  // The inliers object should now contain all the indexes of the points that are part of the floor 
  // create the extact object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (point_cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*floorless_point_cloud);
}

//! Given a Point Cloud, extract the clusters as separate Point Clouds.
/**
 * Given a Point Cloud extract the clusters and put them in their own Point Cloud. 
 * \param point_cloud Pointer to the input Point Cloud. The Point Cloud should have the floor surface removed.
 * \param clusters a vector of Pointers to Point Clouds, each Point Cloud represents a cluster.
 * \param cluster_tolerance the distance between two points that indicates when they become their separate cluster.
 */
void BoundingBoxServer::extractClusters(PointCloudPtr point_cloud, std::vector<PointCloudPtr> &clusters, float cluster_tolerance) {
  pcl::search::KdTree<Point>::Ptr search_tree(new pcl::search::KdTree<Point>()); // Used for quickly searching the Point Cloud structure
  search_tree->setInputCloud(point_cloud); // Pointer to the input Point Cloud 

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean;
  euclidean.setInputCloud (point_cloud);
  euclidean.setSearchMethod (search_tree);
  euclidean.setClusterTolerance (cluster_tolerance);
  euclidean.setMinClusterSize (10);
  euclidean.setMaxClusterSize (50000);
  euclidean.extract (cluster_indices);
  
  // cluster_indices is now a vector, where each items is another vector of all the indices that belong to a single cluster

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : it->indices){
      cluster_cloud->push_back ((*point_cloud)[idx]);
    }
    cluster_cloud->width = cluster_cloud->size ();
    cluster_cloud->height = 1;
    cluster_cloud->is_dense = true;
    
    clusters.push_back (cluster_cloud);
  }
}

//! Function to project the Point Cloud on the planar surface.
/** 
 * Will project all the points to have a zero z value. 
 * \param point_cloud Pointer to the input Point Cloud.
 * \param projected_point_cloud Pointer to the Point Cloud that should contain the projected point_cloud on the planar surface.
 */
void BoundingBoxServer::projectPointCloudOnPlane(PointCloudPtr point_cloud, PointCloudPtr projected_point_cloud) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = coefficients->values[1] = coefficients->values[3] = 0.0f;
  coefficients->values[2] = 1.0f; // Z-axis 

  // Use the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setInputCloud (point_cloud);
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setModelCoefficients (coefficients);
  proj.filter (*projected_point_cloud);
}

//! Get the eigen vectors of a Point Cloud.
/**
 * Assumes the Point Cloud has been projected to the planar surface in order to work correctly.
 * \param point_cloud Pointer to a Point Cloud that has been project on a planar surface.
 * \return A matrix containing the 3 eigen vectors in each column.
 */ 
Eigen::Matrix3f BoundingBoxServer::getEigenVectors(PointCloudPtr point_cloud) {
  
  // Use the PCA object
  pcl::PCA<pcl::PointXYZ> eigenVectors;
  eigenVectors.setInputCloud (point_cloud);
  return eigenVectors.getEigenVectors();
}

//! Get the angle between the eigen vector and a (1,0) vector.
/** 
 * Calculate the angle between two vectors, base_vector is a vector (1,0) representing the direction of the arm base.
 * \param eigen_vector the eigen vector from the object representing the yaw rotation.
 * \return the angle between the base_vector (1,0) and the eigen_vector.
 */
float BoundingBoxServer::getAngle(Eigen::Vector3f eigen_vector) {
  Eigen::Vector2f object_vector = eigen_vector.head<2>();
  Eigen::Vector2f base_vector;
  base_vector << 1.0f, 0.0f; // x = 1.0, y = 0.0

  float angle = atan2(object_vector.y(),object_vector.x()) - atan2(base_vector.y(),base_vector.x());

  /* 
  Returns the angle between the object_vector and base_link_vector.
  Note: the angle can be between -pi and pi. 
  */
 return angle;
}

//! Get the center point of the given Point Cloud.
/**
 * Calculate and return the center point (x, y, z) of the given Point Cloud.
 * \param point_cloud Pointer to the input Point Cloud.
 * \return vector containing the center x, y, z coordinates.
 */
Eigen::Vector3f BoundingBoxServer::getCenterPointCloud(PointCloudPtr point_cloud) {
  Eigen::Vector3f centroid_vector;
  Point pointMin, pointMax;

  pcl::getMinMax3D (*point_cloud, pointMin, pointMax);

  centroid_vector.x() = pointMin.x+(pointMax.x-pointMin.x)/2;
  centroid_vector.y() = pointMin.y+(pointMax.y-pointMin.y)/2;
  centroid_vector.z() = pointMin.z+(pointMax.z-pointMin.z)/2;

  return centroid_vector;
}

//! Gets the dimensions length, width, height of the given Point Cloud.
/**
 * Calculates the min and max points, from which it extract the center of each axis.
 * \param point_cloud Pointer to the input Point Cloud of which to calculate the dimensions, the Point Cloud should have been transformed to the origin.
 * \param bounding_box the return value to include the length, width, and height of point_cloud.
 */
void BoundingBoxServer::getDimensions(PointCloudPtr point_cloud, bounding_box_server::BoundingBox &bounding_box) {
  Point pointMin, pointMax;
  
  pcl::getMinMax3D (*point_cloud, pointMin, pointMax);
  bounding_box.length = pointMax.x - pointMin.x;
  bounding_box.width = pointMax.y - pointMin.y;
  bounding_box.height = pointMax.z - pointMin.z;
}
