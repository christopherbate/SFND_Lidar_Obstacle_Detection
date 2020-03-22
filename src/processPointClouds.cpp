// PCL lib Functions for processing point clouds
#include "processPointClouds.h"
#include "kdtree.h"
#include <random>
#include <unordered_set>

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

    CloudPtr<PointT> vgoCloud(new pcl::PointCloud<PointT>);
    CloudPtr<PointT> localCloud(new pcl::PointCloud<PointT>);

    // ROI Filter first for speed
    pcl::CropBox<PointT> roi;
    roi.setInputCloud(cloud);
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setNegative(false);
    roi.filter(*localCloud);

    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(localCloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.filter(*vgoCloud);

    // Remove the roofpoints
    pcl::CropBox<PointT> rf;
    rf.setInputCloud(vgoCloud);
    rf.setMin(Eigen::Vector4f{-2.25, -1.5f, -1.1f, 1.0f});
    rf.setMax(Eigen::Vector4f{3.5f, 1.5f, 0.0f, 1.0f});
    rf.setNegative(true);
    rf.filter(*localCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return localCloud;
}

template <typename PointT>
CloudPtrPair<PointT> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, CloudPtr<PointT> cloud)
{
    CloudPtr<PointT> planeCloud(new pcl::PointCloud<PointT>);
    CloudPtr<PointT> objectCloud(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
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
CloudPtrPair<PointT> ProcessPointClouds<PointT>::SegmentPlane(CloudPtr<PointT> const &cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Custom Ransac
    // C++ STL Uniform generator
    auto numPoints = cloud->size();
    std::default_random_engine generator;
    std::uniform_int_distribution<int> unidist(0, numPoints - 1);

    // Cross Product of rays
    auto planeCoeff = [](PointT const &ptA, PointT const &ptB, PointT const &ptC) {
        float v1[3]{
            ptB.x - ptA.x, ptB.y - ptA.y, ptB.z - ptA.z};
        float v2[3]{
            ptC.x - ptA.x, ptC.y - ptA.y, ptC.z - ptA.z};
        auto normal = pcl::PointXYZ(
            v1[1] * v2[2] - v1[2] * v2[1],
            -(v1[0] * v2[2] - v1[2] * v2[0]),
            v1[0] * v2[1] - v1[1] * v2[0]);
        auto offset = -(normal.x * ptA.x + normal.y * ptA.y + normal.z * ptA.z);
        return std::make_pair(normal, offset);
    };

    // For max iterations
    std::unordered_set<int> best_set;
    for (int i = 0; i < maxIterations; i++)
    {
        // Randomly sample subset and fit line
        int idx1 = unidist(generator);
        int idx2 = unidist(generator);
        int idx3 = unidist(generator);
        auto pt1 = cloud->at(idx1);
        auto pt2 = cloud->at(idx2);
        auto pt3 = cloud->at(idx3);

        // This is also cross product in homogenous coordinates
        auto planeData = planeCoeff(pt1, pt2, pt3);
        auto plane = planeData.first;
        auto offset = planeData.second;                

        // Calc inlier set, keeping track of best        
        std::unordered_set<int> inliers;
        float normal_mag = std::sqrt(plane.x * plane.x + plane.y * plane.y + plane.z * plane.z);
        float distAdj = normal_mag*distanceThreshold;

        // Check repeat sample case
        if (normal_mag == 0.0f)
        {
            continue;
        }

        for (int j = 0; j < cloud->size(); j++)
        {
            auto pt = cloud->at(j);
            float dist = std::abs(plane.x * pt.x + plane.y * pt.y + plane.z * pt.z + offset);
            if (dist < distAdj)
            {
                inliers.insert(j);
            }
        }

        if (inliers.size() > best_set.size() || i == 0)
        {            
            best_set = inliers;
        }
    }
    CloudPtr<PointT> cloudInliers(new pcl::PointCloud<PointT>());
    CloudPtr<PointT> cloudOutliers(new pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->size(); index++)
    {
        if (best_set.find(index) == best_set.end())
        {
            cloudInliers->push_back(cloud->at(index));
        }
        else
        {
            cloudOutliers->push_back(cloud->at(index));
        }
    }
    std::cout << "Total " << cloud->size() << " Objects " << cloudInliers->size() << " Plane " << cloudOutliers->size() << "\n";
    CloudPtrPair<PointT> segResult(cloudInliers, cloudOutliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

template <typename PointT>
CloudPtrVec<PointT> ProcessPointClouds<PointT>::Clustering(CloudPtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();

    // Custom clustering
    auto kd = std::make_shared<KdTree<PointT>>();
    kd->setInputCloud(cloud);
    auto clusters = kd->euclideanCluster(cloud, clusterTolerance, minSize, maxSize);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters\n";

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