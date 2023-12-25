#include "bounding_box_server/bounding_box_server.h"

//! Filters parts of the Point Cloud per axis based on min and max values.
/**
 * Uses the pcl::PassThrough filter to filter out parts of the Point Cloud. 
 * All points on the specified axis between min_value and max_value will be contained in the output Point Cloud, 
 * point below min_value and above max_value will therefore be removed.
 * \param point_cloud Pointer to input Point Cloud.
 * \param filtered_point_cloud Pointer to the output filtered Point Cloud.
 * \param field_name the axis on which to filter, either "x", "y", or "z".
 * \param min_value the min value off all the filtered points.
 * \param max_value the max value off all the filtered points.
 */
void BoundingBoxServer::passThroughFilter(PointCloudPtr point_cloud, PointCloudPtr filtered_point_cloud,
                                          std::string field_name, float min_value, float max_value) {
  pass_through_filter_.setInputCloud(point_cloud); // Pointer to the input Point Cloud
  pass_through_filter_.setFilterFieldName(field_name); // Indicates on which axis to filter
  pass_through_filter_.setFilterLimits(min_value, max_value); 
  pass_through_filter_.filter(*filtered_point_cloud);  // The output of the filter                               
}

//! Voxelizes the Point Cloud.
/**
 * Reduces the Point Cloud size by creating voxels of multiple points based on the leaf size, each points within 
 * the voxel will be indicated just by that one voxel. This will mostly help to reduce the calculation time of other 
 * algorithms.
 * \param point_cloud Pointer to the Point Cloud.
 * \param voxelized_point_cloud Pointer to the voxelized Point Cloud based on leaf_size parameter.
 * \param leaf_size indices how large the voxels are.
 */
void BoundingBoxServer::voxelizePointCloud(PointCloudPtr point_cloud, PointCloudPtr voxelized_point_cloud, float leaf_size) {
  voxel_grid_.setInputCloud(point_cloud); // Pointer to the input Point Cloud
  voxel_grid_.setLeafSize(leaf_size, leaf_size, leaf_size); // The leaf size of the voxel, each point in this voxel will be indicated by 1 point
  voxel_grid_.filter(*voxelized_point_cloud); // The reduced Point Cloud 
}