#ifndef POINTCLOUD_H
#define POINTCLOUD_H
#include <opencv2/opencv.hpp>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Eigen>
#include <Eigen/Core>

class PointCloud
{
public:
    PointCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud2imu0
        (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , cv::Mat Tvel2imu0);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr CalcNormal(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorPointCloudFromImage
            (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud , cv::Mat Tvel2cam , cv::Mat image);
    void Smoothing(
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &smoothed_cloud);
    void Ground_Object_disscrimination(
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud ,
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &grourd_cloud ,
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &nonGround_cloud  );
};

#endif // POINTCLOUD_H
