// PCL lib Functions for processing point clouds
#include "processPointClouds.h"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(CloudPtr<PointT> cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
CloudPtr<PointT> ProcessPointClouds<PointT>::FilterCloud(CloudPtr<PointT> cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;
}

template <typename PointT>
CloudPtrPair<PointT> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, CloudPtr<PointT> cloud)
{
    CloudPtr<PointT> planeCloud(new pcl::PointCloud<PointT>);
    CloudPtr<PointT> objectCloud(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*planeCloud);

    extract.setNegative(true);
    extract.filter(*objectCloud);

    CloudPtrPair<PointT> segResult(objectCloud, planeCloud);
    return segResult;
}

template <typename PointT>
CloudPtrPair<PointT> ProcessPointClouds<PointT>::SegmentPlane(CloudPtr<PointT> cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    auto segResult = SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds; found plane of size " << segResult.second->size() << "; objects of size " << segResult.first->size() << std::endl;

    return segResult;
}

template <typename PointT>
CloudPtrVec<PointT> ProcessPointClouds<PointT>::Clustering(CloudPtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize)
{    
    auto startTime = std::chrono::steady_clock::now();    

    // Perform clustering
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMaxClusterSize(maxSize);
    ec.setMinClusterSize(minSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Seperate the clusters into the vector
    CloudPtrVec<PointT> clusters;
    for (auto indices : cluster_indices)
    {
        CloudPtr<PointT> cloud_cluster(new pcl::PointCloud<PointT>);
        for (auto idx : indices.indices)
        {
            cloud_cluster->points.push_back(cloud->points[idx]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(CloudPtr<PointT> cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(CloudPtr<PointT> cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    CloudPtr<PointT> cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;
    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}