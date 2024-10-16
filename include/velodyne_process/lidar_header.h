#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <unordered_set>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pcl/point_types.h>

/* #include <mlpack.hpp>
#include <mlpack/methods/dbscan/dbscan.hpp> */
#include <armadillo>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>