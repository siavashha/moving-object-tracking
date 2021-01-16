#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <boost/lexical_cast.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
//#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/interactor_style.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

class Visualizer
{
private:
    int object_counter;

public:
    Visualizer(pcl::visualization::PCLVisualizer& viewer);
    Visualizer(pcl::visualization::PCLVisualizer& viewer , cv::Mat orientation);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ,
                       std::string color);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr cloud ,
                       cv::Point3f color );
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr cloud ,
                       int k);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                       cv::Mat b);
    void ShowCoordinateSystem(pcl::visualization::PCLVisualizer& viewer ,
                       std::vector<cv::Mat> NED);
    void ShowRoadLinks(pcl::visualization::PCLVisualizer& viewer,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr roadlinkcloud);
    cv::Point3f GroundColorMix( float x, float min, float max);

    void ProjectedPCD_Vis(std::string imgFilename ,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_PCD);
    void FindMinMaxPCD(float& max , float& min, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_PCD);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ,
                                   pcl::PointCloud<pcl::Normal>::Ptr normals);

};

#endif // VISUALIZER_H
