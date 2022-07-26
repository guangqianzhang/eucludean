//
// Created by zgq on 2022/5/14.
//
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include "src/render.h"
#include "src/ProcessPointClouds.h"
#include "src/ProcessPointClouds.cpp"//moban

using namespace  std;
typedef pcl::PointXYZ point;
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}
template<typename PointT>
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<PointT>* PointProcessorI,
               const typename pcl::PointCloud<PointT>::Ptr& inputCloud){
    float filterRes = 0.4;
    Eigen::Vector4f minpoint(-10, -6.5, -2, 1);
    Eigen::Vector4f maxpoint(30, 6.5, 1, 1);
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered = PointProcessorI->FilterCloud(inputCloud,0.3,
                                                                                       minpoint,
                                                                                       maxpoint);
//    renderPointCloud(viewer,cloudFiltered,"cloudFiltered",Color(200,0,0));

    std::pair<pcl::PointCloud<point>::Ptr, pcl::PointCloud<point>::Ptr> segmentCloud
                                                                = PointProcessorI->SegmentPlane(cloudFiltered, 50, 0.3);
//    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
//    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    //clustering
    std::vector<pcl::PointCloud<point>::Ptr> cloudClusters = PointProcessorI->Clustering(segmentCloud.first, 0.53, 10, 500);
    renderPointCloud(viewer,cloudClusters[2],"obstCloud",Color(1,0,0));
    std::cout<<"cloud size: "<<cloudClusters.size()<<std::endl;

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<point>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        PointProcessorI->numPoints(cluster);
        //renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        std::cout<<"ClusterId: " << clusterId<<std::endl;
        //render box;
        Box box = PointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}


int main(int argc, char **argv) {
    cout<<"hallo world!"<<endl;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = Side;
    initCamera(setAngle,viewer);

    //Stream PCD
    ProcessPointClouds<point>* pointProcessorI = new ProcessPointClouds<point>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("/home/zgq/Documents/dataset/2011_09_26/pcd");
    auto streamIterator = stream.begin();
    pcl::PointCloud<point>::Ptr inputCloudI;


    while (!viewer->wasStopped()){
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        if(inputCloudI->empty()){
            streamIterator++;
            if(streamIterator == stream.end())
                streamIterator = stream.begin();
            continue;}
//        renderPointCloud(viewer,inputCloudI,"inputCloud");

        cityBlock(viewer, pointProcessorI, inputCloudI);
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce ();
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

}
