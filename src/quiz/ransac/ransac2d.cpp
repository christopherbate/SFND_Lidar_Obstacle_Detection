/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include <random>
#include "../../processPointClouds.h"
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	srand(time(NULL));

	// C++ STL Uniform generator	
	auto numPoints = cloud->width;
	std::default_random_engine generator;
	std::uniform_int_distribution<int> unidist(0, numPoints - 1);

	// Cross Product of rays
	auto planeCoeff = [](pcl::PointXYZ const &ptA, pcl::PointXYZ const &ptB, pcl::PointXYZ const &ptC) {
		float v1[3]{
			ptB.x - ptA.x, ptB.y - ptA.y, ptB.z - ptA.z};
		float v2[3]{
			ptC.x - ptA.x, ptC.y - ptA.y, ptC.z - ptA.z};
		auto normal = pcl::PointXYZ(
			v1[1] * v2[2] - v1[2] * v2[1],
			-(v1[0] * v2[2] - v1[2] * v2[0]),
			v1[0] * v2[1] - v1[1] * v2[0]);
		auto offset = -(normal.x * ptA.x + normal.y * ptB.y + normal.z * ptC.z);
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
		// Note: ignore the denominator for dist. fn as normalization here doesn't matter,
		// and it's best to avoid dividing small numbers.
		std::unordered_set<int> inliers;
		float normal_mag = std::sqrt(plane.x * plane.x + plane.y * plane.y + plane.z * plane.z);

		// Check repeat sample case
		if (normal_mag == 0.0f)
		{
			continue;
		}

		float distTolAdjusted = distanceTol * normal_mag;
		for (int j = 0; j < cloud->width; j++)
		{
			auto pt = cloud->at(j);
			float dist = std::abs(plane.x * pt.x + plane.y * pt.y + plane.z * pt.z) + offset;
			if (dist < distTolAdjusted)
			{
				inliers.insert(j);
			}
		}
		if (inliers.size() > best_set.size() || i == 0)
		{
			best_set = inliers;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	return best_set;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());
	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
