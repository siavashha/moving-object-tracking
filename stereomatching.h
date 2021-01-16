#ifndef STEREOMATCHING_H
#define STEREOMATCHING_H
#include <pcl/stereo/stereo_matching.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/io/png_io.h>
//#include <pcl/io/pcd_grabber.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/pcl_base.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>

class StereoMatching
{
public:
    StereoMatching();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Matcher
        (pcl::PointCloud<pcl::RGB>::Ptr lImage_PCD  ,
         pcl::PointCloud<pcl::RGB>::Ptr rImage_PCD);
    pcl::PointCloud<pcl::RGB>::Ptr Reader(std::string lImage_filename);


};

#endif // STEREOMATCHING_H
