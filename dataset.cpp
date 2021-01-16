#include "dataset.h"

Dataset::Dataset(std::string KITTI_DIR)
{
    // camera data
    std::string KITTI_leftCam_DIR = KITTI_DIR + "image_00";
    std::string KITTI_rightCam_DIR = KITTI_DIR + "image_01";
    std::string KITTI_leftCam_Color_DIR = KITTI_DIR + "image_02";
    std::string KITTI_rightCam_Color_DIR = KITTI_DIR + "image_03";
    // IMU data
    std::string KITTI_IMU_Dir = KITTI_DIR + "oxts/data/";
    // Laser Scanner data
    std::string KITTI_Laser_Scanner_Dir = KITTI_DIR + "velodyne_points";
    // calibration files
    std::string KITTI_Cam_Calib = KITTI_DIR + "calib_cam_to_cam.txt";
    std::string KITTI_IMU_VEL = KITTI_DIR + "calib_imu_to_velo.txt";
    std::string KITTI_VEL_CAM = KITTI_DIR + "calib_velo_to_cam.txt";
    // road link
    std::string KITTI_LINK_ROAD = KITTI_DIR + "RoadLink.txt";

    int i = 0;
    for (boost::filesystem::directory_iterator itr(KITTI_IMU_Dir);
         itr!=boost::filesystem::directory_iterator() ; ++itr )
    {
    // reading the image seqeuces
    std::ostringstream convert;
    convert << i;
    std::string firstDigits = convert.str();
    std::string KITTI_filename;
    if (firstDigits.length() == 1)
        KITTI_filename = "000000000" + firstDigits ;
    if (firstDigits.length() == 2)
        KITTI_filename = "00000000" + firstDigits ;
    if (firstDigits.length() == 3)
        KITTI_filename = "0000000" + firstDigits ;
    if (firstDigits.length()>3 || firstDigits.length()==0)
        std::fprintf(stderr,"error in naming");

    // reading camera files
    std::string KITTI_leftImage_BW = KITTI_leftCam_DIR + "/data/" + KITTI_filename + ".png";
    std::string KITTI_rightImage_BW = KITTI_rightCam_DIR + "/data/" + KITTI_filename + ".png";
    std::string KITTI_leftImage_Color = KITTI_leftCam_Color_DIR + "/data/" + KITTI_filename + ".png";
    std::string KITTI_rightImage_Color = KITTI_rightCam_Color_DIR + "/data/" + KITTI_filename + ".png";
    // reading laser scanner and IMU
    std::string KITTI_laserScanner = KITTI_Laser_Scanner_Dir + "/data/" + KITTI_filename + ".bin";
    std::string KITTI_IMU_filename = KITTI_IMU_Dir + KITTI_filename + ".txt";
    roadLink_filename = KITTI_LINK_ROAD;
    calib_filename = KITTI_Cam_Calib;
    lImages_bw_list.push_back(KITTI_leftImage_BW);
    rImages_bw_list.push_back(KITTI_rightImage_BW);
    lImages_color_list.push_back(KITTI_leftImage_Color);
    rImages_color_list.push_back(KITTI_rightImage_Color);
    laserScanner_list.push_back(KITTI_laserScanner);
    imu_list.push_back(KITTI_IMU_filename);
    i++;
    }

}

sensor_data Dataset::LoadData(int epoch)
{
    sensor_data data;
    data.cloud = ReadPCD(laserScanner_list[epoch]);
    std::vector<cv::Mat> imu_data = ReadIMU(imu_list[epoch]);
    //std::vector<cv::Mat> calib = Camera_Calibration(std::string calib_filename);
    data.imu_pos_llh = imu_data[0];
    data.imu_vel_NE = imu_data[1];
    data.imu_angle_RPY = imu_data[2];
    data.imu_vel_FLU = imu_data[3];
    data.imu_AngularVel_FLU = imu_data[4];

    cv::Mat mTimu2vel = Timu2vel();
    data.Timu2cam = mTimu2vel;

    cv::Mat mTvel2cam = Tvel2cam();
    data.Tvel2cam = mTvel2cam;

    std::vector<cv::Mat> proj = CameraProjectionMatrix();
    data.lImgBW_projection = proj[0];
    data.rImgBW_projection = proj[1];
    data.lImgColor_projection = proj[2];
    data.rImgColor_projection = proj[3];

    data.lImgColor = cv::imread(lImages_color_list[epoch]);
    data.rImgColor = cv::imread(rImages_color_list[epoch]);

    std::vector<cv::Mat> camera_calib = Camera_Calibration(calib_filename,
                                        data.lImgColor.cols , data.lImgColor.rows );
    data.lImgBW_calib = camera_calib[0];
    data.rImgBW_calib = camera_calib[1];
    data.lImgColor_calib = camera_calib[2];
    data.rImgColor_calib = camera_calib[3];    

    pcl::PointCloud <pcl::PointXYZ>::Ptr roadLink = RoadLinkLoader(roadLink_filename);
    data.llh_road_links = roadLink;

    return data;
}

