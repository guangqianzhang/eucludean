//
// Created by zgq on 2022/5/14.
//

#ifndef TEST5_PROCESSPOINTCLOUDS_H
#define TEST5_PROCESSPOINTCLOUDS_H
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <unordered_set>
#include "Box.h"
#include "KDTree.h"

template<typename PointT>
class ProcessPointClouds {
public:
    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);
    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);
    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                      float filterRes, Eigen::Vector4f minPoint,
                                                      Eigen::Vector4f maxPoint);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers,
                                                                                                           typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                                         int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);



    void clusterHelper(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize);

};


#endif //TEST5_PROCESSPOINTCLOUDS_H
