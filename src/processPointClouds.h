// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>

#include "render/box.h"
#include "types.h"

template <typename PointT>
class ProcessPointClouds
{
public:
    ProcessPointClouds();
    ~ProcessPointClouds();

    void numPoints(CloudPtr<PointT> cloud);

    CloudPtr<PointT> FilterCloud(CloudPtr<PointT> cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    /**
     * Takes inliers for some model (in this case, plane model), and seperates
     * inliers in "cloud" from the rest, returning the seperated clouds
     * 
     * Returns: CloudPtrPair (object cloud, plane cloud)
    **/
    CloudPtrPair<PointT> SeparateClouds(pcl::PointIndices::Ptr inliers, CloudPtr<PointT> cloud);

    /**
     * Performs RANSAC in order to seperate the groud plane from the rest of the cloud.
     */
    CloudPtrPair<PointT> SegmentPlane(CloudPtr<PointT> const& cloud, int maxIterations, float distanceThreshold);

    /** 
     * Performs K-d clustering on cloud (typically non-plane points) to identify obstacles.
     * Returns vector of CloudPtr, each an estimate obstacle
     */
    CloudPtrVec<PointT> Clustering(CloudPtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(CloudPtr<PointT> cluster);

    void savePcd(CloudPtr<PointT> cloud, std::string file);

    CloudPtr<PointT> loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};
#endif /* PROCESSPOINTCLOUDS_H_ */