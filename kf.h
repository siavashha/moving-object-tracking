#ifndef KF_H
#define KF_H
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>

struct object_cluster
{
    int unique_key;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    cv::Mat pos;//(3,1,CV_64F, cv::Scalar(0.))
//    cv::Matx31f pos_dot;
//    cv::Matx31f rot;
//    cv::Matx31f rot_dot;
//    bool is_static;
//    cv::KalmanFilter kalman;
};

class KF
{
private:

public:
    KF();
    void KF_initialization(std::vector<object_cluster>& objects_clusters ,
              std::vector<pcl::PointCloud <pcl::PointXYZI>::Ptr> pre_clusters ,
              std::vector<pcl::PointCloud <pcl::PointXYZI>::Ptr> clusters);
    void Object_Initialization(std::vector<object_cluster>& objects_clusters ,
                std::vector<pcl::PointCloud <pcl::PointXYZI>::Ptr> clusters);
};

#endif // KF_H
