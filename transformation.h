#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H
//#define 	DEG2RAD(x)   ((x) * 0.01745329251994329575)
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include "SensorsData.h"
class Transformation
{
private:
    cv::Mat ENU0;
    cv::Mat Cimu0_nav;
    double primeVCosined , meridianCurv;


    std::vector<cv::Mat> m_pos;
    std::vector<cv::Mat> m_velFLU;
    std::vector<cv::Mat> m_orient;
    std::vector<cv::Mat> m_V_ne;
    std::vector<cv::Mat> m_angVel;
    std::vector<double> m_timeStamps;
    cv::Mat m_Cimu2cam;
    cv::Mat m_bimu2cam;
public:
    Transformation();
    void CreateOrigin(sensor_data data);
    cv::Mat Timu2imu0(sensor_data data);
    cv::Mat ConvertToVelRef(int epoch, int ref);

    int ConvertToCamRef(int epoch, int ref);
    int ConvertToCamRef(const cv::Mat llh , cv::Mat& xyz , int ref);
    cv::Mat TranslationInNav(int epoch, int ref);
    pcl::PointCloud <pcl::PointXYZ>::Ptr RoadLink
        (sensor_data data);

    cv::Mat ConvertNav2Imu(cv::Mat EulerAngles); // navigation coordinate system to imu coordinate system
    cv::Mat ConvertImu2Nav(cv::Mat EulerAngles); // imu coordinate system to navigation coordinate system
    cv::Mat R1(double th);
    cv::Mat R2(double th);
    cv::Mat R3(double th);

    std::vector< cv::Mat> EpipoleComposer(cv::Mat calib, int epoch);
    cv::Mat FundamentalComposer(cv::Mat calib, int epoch);

    std::vector<cv::Mat> CoordinateSystem();

    // IMU observations in reference camera epoch (=1)
    double dt;
    double dt_i;
    cv::Mat CvelRef2vel;
    cv::Mat vel0_vel;
    cv::Mat xVelRef;
    cv::Mat vVelRef;
    cv::Mat OmgVelRef;
    cv::Mat vel0_vel_i;
    cv::Mat x_VelRef_i;
};

#endif // TRANSFORMATION_H
