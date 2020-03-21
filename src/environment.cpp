/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
#include <memory>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"

/**
 * Sets up the scene within the PCL vizualizer
 */
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

/**
 * Open 3D viewer and display simple highway
 */ 
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // Perform lidar scan
    auto lidarPtr = std::make_shared<Lidar>(cars, 0.0);
    auto scan = lidarPtr->scan();    

    // Segment ground plane then cluster
    auto pclProcessor = std::make_shared<ProcessPointClouds<pcl::PointXYZ>>();
    auto clouds = pclProcessor->SegmentPlane(lidarPtr->cloud, 100, 0.2);
    auto clusters = pclProcessor->Clustering(clouds.first, 2.0, 2, 1000);

    // Render clusters in different colors
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};    
    for (auto cluster : clusters)
    {        
        pclProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % 3]);
        auto box = pclProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }

    // Render ground plane
    // renderPointCloud(viewer, clouds.second, "ground plane", Color(0, 1, 0));
}

/**
 * setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
 */
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    viewer->setBackgroundColor(0, 0, 0);    
    viewer->initCameraParameters();    
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}