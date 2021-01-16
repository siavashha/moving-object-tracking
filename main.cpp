#include <QCoreApplication>
#include <stdlib.h>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

//#include "laserscanner.h"
#include "transformation.h"
#include "visualizer.h"
#include "dataset.h"
#include "SensorsData.h"
#include "pointcloud.h"
#include "stereomatching.h"
#include <time.h>
#include <Eigen/Eigen>
#include <Eigen/Core>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    // KITTI dataset directory
    std::string  KITTI_DIR =  "/home/sia/Documents/Data/2011_09_26/2011_09_26_drive_0005_sync/";
    //Data is loaded in vectors and matrices
    Dataset* datasetReader = new Dataset(KITTI_DIR);
    cv::Mat Timu2vel = datasetReader->Timu2vel();
    cv::Mat Tvel2cam = datasetReader->Tvel2cam();
    cv::Mat Tvel2lCamBW = datasetReader->Tvel2cam();
    Transformation* trans = new Transformation();
    PointCloud* ls = new PointCloud();
    StereoMatching* stereoMatcher = new StereoMatching();
    //Create Origin
    sensor_data data0 = datasetReader->LoadData(0);
    trans->CreateOrigin(data0);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
            (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    Visualizer* viz = new Visualizer(*viewer);
    std::vector<cv::Mat> ENU = trans->CoordinateSystem();
    viz->ShowCoordinateSystem(*viewer , ENU);
    pcl::PointCloud <pcl::PointXYZ>::Ptr roalLink =
            trans->RoadLink (data0);
    viz->ShowRoadLinks(*viewer , roalLink);
    //   std::cout << roalLink->points[0] << std::endl;
    int firstEpoch = 0;
    int last_Epoch = 152;
    for (int i = 1 ; i < last_Epoch - firstEpoch; i++)
    {
        sensor_data data = datasetReader->LoadData(i);
                std::string lImgFilename = datasetReader->lImages_color_list[i];
                std::string rImgFilename = datasetReader->rImages_color_list[i];
                pcl::PointCloud<pcl::RGB>::Ptr lStereo_cloud =
                        stereoMatcher->Reader(lImgFilename);
                pcl::PointCloud<pcl::RGB>::Ptr rStereo_cloud =
                        stereoMatcher->Reader(rImgFilename);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr stereo_cloud =
                        stereoMatcher->Matcher(lStereo_cloud , rStereo_cloud);
        cv::Mat Tvel2lCamColor = data.lImgColor_projection * Tvel2lCamBW;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud =ls->ColorPointCloudFromImage
                (data.cloud , Tvel2lCamColor , data.lImgColor);
        // transform LS & stereo
        cv::Mat Timu2imu0 = trans->Timu2imu0(data);
        cv::Mat Tvel2imu0 =  Timu2imu0 * Timu2vel.inv();
//        std::cout << "Timu2imu0 : " << Timu2imu0 <<
//                     " Timu2vel : " << Timu2vel << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcdInImu0 =
                ls->PointCloud2imu0( rgbCloud , Tvel2imu0);
        cv::Mat Tcam2imu0 = Tvel2imu0 * Tvel2cam.inv();
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr stereoInImu0 =
                        ls->PointCloud2imu0( stereo_cloud , Tvel2imu0);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudRGBNormal
                (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        cloudRGBNormal = ls->CalcNormal(rgbCloud);
//        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr smoothed_cloud;
//        ls->Smoothing( cloudRGBNormal , smoothed_cloud);
        viz->AddPointCloud(*viewer , cloudRGBNormal);
        viz->AddPointCloud(*viewer , stereoInImu0);
        //viz->AddPointCloud(*viewer , rgbCloud , cloudNormal);


//                // Segment Points on the Ground
//                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ground_cloud
//                        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
//                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr nonGround_cloud
//                        (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
//                ls->Ground_Object_disscrimination
//                        (cloudRGBNormal , ground_cloud , nonGround_cloud);
//                viz->AddPointCloud(*viewer , nonGround_cloud);

//                pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud
//                        (new pcl::PointCloud<pcl::PointXYZI>());

//                transformed_cloud = ls->PointCloud2imu0(data.cloud , Tvel2lCamColor);




        //viz->AddPointCloud(*viewer , rgbCloud);
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (1000));
        }
        viewer->close();

    }
    return a.exec();
}
