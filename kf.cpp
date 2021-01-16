#include "kf.h"

KF::KF()
{
}

void KF::KF_initialization(std::vector<object_cluster>& objects_clusters ,
            std::vector<pcl::PointCloud <pcl::PointXYZI>::Ptr> pre_clusters ,
            std::vector<pcl::PointCloud <pcl::PointXYZI>::Ptr> clusters)
{
    for (int i = 0 ; i < objects_clusters.size(); i ++)
    {
        object_cluster a;// = objects[i];
        pcl::PointCloud <pcl::PointXYZI>::Ptr cloud = clusters[i];

        a.cloud = cloud;
        Eigen::Vector4f cloud_center ;
        pcl::compute3DCentroid(*cloud  , cloud_center );
        a.pos;//(3,1,CV_32F , cv::Scalar(0.));
        a.pos.at<float>(0,0) = cloud_center(0);
        a.pos.at<float>(1,0) = cloud_center(1);
        a.pos.at<float>(2,0) = cloud_center(2);


        //a.kalman
    }
}

void KF::Object_Initialization(std::vector<object_cluster>& objects_clusters ,
            std::vector<pcl::PointCloud <pcl::PointXYZI>::Ptr> clusters)
{
    for (int i = 0 ; i < clusters.size(); i ++)
    {
        object_cluster a;// = objects[i];
        pcl::PointCloud <pcl::PointXYZI>::Ptr cloud = clusters[i];
        a.unique_key = i;
        a.cloud = cloud;
        Eigen::Vector4f cloud_center;
        pcl::compute3DCentroid(*cloud  , cloud_center );
        cv::Mat position(3,1,CV_64F,cv::Scalar(0.));
        position.at<double>(0,0) = cloud_center(0);
        position.at<double>(1,0) = cloud_center(1);
        position.at<double>(2,0) = cloud_center(2);
        a.pos = position;
        objects_clusters.push_back(a);
    }
}

