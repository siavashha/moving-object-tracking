#ifndef SENSORSDATA_H
#define SENSORSDATA_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct sensor_data{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    cv::Mat imu_pos_llh, imu_vel_NE, imu_angle_RPY , imu_vel_FLU, imu_AngularVel_FLU;
    cv::Mat lImgColor , rImgColor;
    cv::Mat lImgColor_projection , rImgColor_projection , lImgBW_projection , rImgBW_projection;
    cv::Mat Tvel2cam , Timu2cam;
    cv::Mat lImgBW_calib , rImgBW_calib , lImgColor_calib , rImgColor_calib;
    pcl::PointCloud <pcl::PointXYZ>::Ptr llh_road_links;
    cv::KalmanFilter kf;
};
#endif // SENSORSDATA_H
