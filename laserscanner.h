#ifndef LASERSCANNER_H
#define LASERSCANNER_H
#include <iostream>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
//#include <Eigen/Dense>
//#include <Eigen/Array>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/point_cloud.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/correspondence.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/tracker.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>


//#include <pcl/keypoints/impl/iss_3d.hpp>
//#include <pcl/keypoints/iss_3d.h>

#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/hough_3d.h>

#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>


#include "DataLoader.h"

class LaserScanner
{
private:
    std::vector <std::string> m_ls_list;
public:
    LaserScanner();
    void ReadPCD(std::string filename , pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool filtered);
    void Ground_Object_disscrimination(
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud ,
            pcl::PointCloud<pcl::PointXYZI>::Ptr &grourd_cloud ,
            pcl::PointCloud<pcl::PointXYZI>::Ptr &nonGround_cloud) ;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr Filter( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    std::vector < pcl::PointCloud<pcl::PointXYZI>::Ptr > Clustering(
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud2imu0
    (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud , cv::Mat Tvel2imu0);
    void onRoad_offRoad_segmentation(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr &road,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr &on_road,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr &off_road);
    pcl::PointCloud<pcl::Normal>::Ptr CalcNormal(
            pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud);
    pcl::PointCloud<pcl::SHOT352>::Ptr  CalcSHOT(
            pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud ,
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normal ,
            pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr CalcFPFH(
            pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud ,
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normal ,
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keypoints);
    pcl::CorrespondencesPtr CalcSHOTCorrespondance(
            pcl::PointCloud<pcl::SHOT352>::Ptr  shot0 ,
            pcl::PointCloud<pcl::SHOT352>::Ptr shot1 );
    pcl::CorrespondencesPtr  CalcFPFHCorrespondance(
            pcl::PointCloud<pcl::FPFHSignature33>::Ptr  fpfh0 ,
            pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh1 );
    pcl::PointCloud<pcl::PointXYZI>::Ptr CalcKeyPoints(
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud  );
    pcl::PointCloud<pcl::PointXYZI>::Ptr CalcSIFTKeyPoints(
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud  );
    bool clusterMatching(std::vector < bool>& visibiity ,
            const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
            std::vector < pcl::PointCloud<pcl::PointXYZI>::Ptr > &clusters,
            std::vector < cv::KalmanFilter >& track_clusters ,
            Eigen::Matrix4f &transform_svd );

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Color_LS
        (const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
          std::string image_filename ,
     cv::Mat C_vel2nav, cv::Mat b_vel2nav,cv::Mat C_vel2cam, cv::Mat b_vel2cam);
    void KF_initialization(cv::KalmanFilter &kf, Eigen::Vector4f center);
    cv::Mat imageProjection(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
             std::string imgFilename ,
             cv::Mat C_vel2nav, cv::Mat b_vel2nav,cv::Mat C_vel2cam, cv::Mat b_vel2cam);

};

#endif // LASERSCANNER_H
