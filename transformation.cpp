#include "transformation.h"

Transformation::Transformation()
{

}

void Transformation::CreateOrigin(sensor_data data)
{
    //epoch time t-1
    double latti0 = data.imu_pos_llh.at<double>(0,0);
    double longi0 = data.imu_pos_llh.at<double>(1,0);
    double h0 = data.imu_pos_llh.at<double>(2,0);
    // translation reference
    double simeMajorAxisOfEarth = 6378137;
    double flattening = 1 / 298.257222101;
    double e2 = 2 * flattening -flattening * flattening;
    primeVCosined = simeMajorAxisOfEarth / std::sqrt(1- e2 * sin(DEG2RAD(latti0)) *
                                                     sin(DEG2RAD(latti0))) ; // N * cos (phi)
    primeVCosined=primeVCosined*cos(DEG2RAD(latti0));
    meridianCurv = simeMajorAxisOfEarth * (1 - e2) /
            ( pow(std::sqrt(
                      1 - e2 * sin(DEG2RAD(latti0)) * sin(DEG2RAD(latti0))
                      ),3.)); // M
    double N0 = meridianCurv * DEG2RAD( latti0);
    double E0 = primeVCosined * DEG2RAD( longi0);
    double U0 = h0;
    ENU0 = cv::Mat::zeros(3,1,CV_64F);
    ENU0.at<double> (0,0) = E0;
    ENU0.at<double> (1,0) = N0;
    ENU0.at<double> (2,0) = U0;
    // East, North, Up (navigation frame)
    cv::Mat Cref_nav = ConvertImu2Nav(data.imu_angle_RPY);
    Cimu0_nav = Cref_nav;
}

cv::Mat Transformation::Timu2imu0(sensor_data data)
{
    cv::Mat Timu2imu0(4,4,CV_64F,cv::Scalar(0.));
    cv::Mat Timu2imu0_34(3,4,CV_64F,cv::Scalar(0.));
    cv::Mat aux = (cv::Mat_<double>(1,4) << 0 , 0 , 0 , 1);
    //epoch time t-1
    double N = meridianCurv * DEG2RAD(data.imu_pos_llh.at<double>(0,0));
    double E = primeVCosined * DEG2RAD( data.imu_pos_llh.at<double>(1,0));
    double U = data.imu_pos_llh.at<double>(2,0);
    cv::Mat ENU(3,1,CV_64F,cv::Scalar(0.));
    ENU.at<double> (0,0) = E;
    ENU.at<double> (1,0) = N;
    ENU.at<double> (2,0) = U;
    cv::Mat dENU = ENU - ENU0;
    //std::cout << " ENU : " << ENU << " ENU0 " << ENU0 << std::endl;
    // East, North, Up (navigation frame)
    cv::Mat Cimu_nav = ConvertImu2Nav(data.imu_angle_RPY);
    cv::Mat Cimu0_imu = Cimu_nav ;//* Cimu0_nav.inv();
    cv::hconcat(Cimu0_imu , dENU , Timu2imu0_34);
    cv::vconcat(Timu2imu0_34 , aux , Timu2imu0);
    return Timu2imu0;
}

cv::Mat Transformation::R3(double th)
{
    cv::Mat rot_z(3,3,CV_64F,cv::Scalar(0.));
    rot_z.at<double>(0,0) = cos(th);
    rot_z.at<double>(0,1) = sin(th);
    rot_z.at<double>(0,2) = 0;
    rot_z.at<double>(1,0) = -sin(th);
    rot_z.at<double>(1,1) = cos(th);
    rot_z.at<double>(1,2) = 0;
    rot_z.at<double>(2,0) = 0;
    rot_z.at<double>(2,1) = 0;
    rot_z.at<double>(2,2) = 1.;
    return rot_z;
}

cv::Mat Transformation::R2(double th)
{
    cv::Mat rot_y(3,3,CV_64F,cv::Scalar(0.));
    rot_y.at<double>(0,0) = cos(th);
    rot_y.at<double>(0,1) = 0;
    rot_y.at<double>(0,2) = -sin(th);
    rot_y.at<double>(1,0) = 0;
    rot_y.at<double>(1,1) = 1.;
    rot_y.at<double>(1,2) = 0;
    rot_y.at<double>(2,0) = sin(th);
    rot_y.at<double>(2,1) = 0;
    rot_y.at<double>(2,2) = cos(th);
    return rot_y;
}

cv::Mat Transformation::R1(double th)
{
    cv::Mat rot_x(3,3,CV_64F,cv::Scalar(0.));
    rot_x.at<double>(0,0) = 1.;
    rot_x.at<double>(0,1) = 0;
    rot_x.at<double>(0,2) = 0;
    rot_x.at<double>(1,0) = 0;
    rot_x.at<double>(1,1) = cos(th);
    rot_x.at<double>(1,2) = sin(th);
    rot_x.at<double>(2,0) = 0;
    rot_x.at<double>(2,1) = -sin(th);
    rot_x.at<double>(2,2) = cos(th);
    return rot_x;
}

cv::Mat Transformation::ConvertImu2Nav(cv::Mat EulerAngles)
{
    // Jekeli 2000 p.26 eq. 1.91 NED
    double roll = EulerAngles.at<double>(0,0);
    double pitch = EulerAngles.at<double>(1,0);
    double yaw = EulerAngles.at<double>(2,0);
    return R3(-yaw) * R2(-pitch) * R1(-roll);
}

cv::Mat Transformation::ConvertNav2Imu(cv::Mat EulerAngles)
{
    cv::Mat C = ConvertImu2Nav( EulerAngles);
    return C.t();
}

pcl::PointCloud <pcl::PointXYZ>::Ptr Transformation::RoadLink
        (sensor_data data)
{
    double E0 = ENU0.at<double> (0,0);
    double N0 = ENU0.at<double> (1,0);
    double U0 = ENU0.at<double> (2,0);
    pcl::PointCloud <pcl::PointXYZ>::Ptr road_links
            ( new pcl::PointCloud <pcl::PointXYZ>() );
    for (int i = 0 ; i < data.llh_road_links->size() ; i++)
    {
        pcl::PointXYZ llh;
        llh = data.llh_road_links->points[i];
        double longi = llh.x;
        double lati = llh.y;
        double height = llh.z;

        double N = meridianCurv * DEG2RAD(lati);
        double E = primeVCosined * DEG2RAD(longi);
        double U = height;
        pcl::PointXYZ p;
        p.x = E-E0;
        p.y = N-N0;
        p.z = -1.5;
        road_links->push_back(p);
    }
    return road_links;
}

std::vector<cv::Mat> Transformation::CoordinateSystem()
{
    std::vector<cv::Mat> ENU;
      cv::Mat north , east , down;
    // rotation reference
    // East, North, Up (navigation frame)
    cv::Mat x_NavE(3,1,CV_64F,cv::Scalar(0));
    x_NavE.at<double>(0,0) = 10;
    x_NavE.at<double>(1,0) = 0;
    x_NavE.at<double>(2,0) = 0;
    east =  x_NavE;
    ENU.push_back(east);    cv::Mat x_NavN(3,1,CV_64F,cv::Scalar(0));
    x_NavN.at<double>(0,0) = 0;
    x_NavN.at<double>(1,0) = 10;
    x_NavN.at<double>(2,0) = 0;
    north =  x_NavN;
    ENU.push_back(north);
    cv::Mat x_NavH(3,1,CV_64F,cv::Scalar(0));
    x_NavH.at<double>(0,0) = 0;
    x_NavH.at<double>(1,0) = 0;
    x_NavH.at<double>(2,0) = 10;
    down =  x_NavH;
    ENU.push_back(down);
    return ENU;
}
