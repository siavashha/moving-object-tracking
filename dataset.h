#ifndef DATASET_H
#define DATASET_H
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "DataLoader.h"
#include "SensorsData.h"


class Dataset : public DataLoader
{
private:
    std::vector<std::string> laserScanner_list; // laser scanner
    std::vector<std::string> imu_list; // imu sequence
    std::vector <double> lTimeStamps; // left image time stamps; go to KITTI dataset
    std::vector <double> rTimeStamps; // right image time stamps; go to KITTI dataset
    std::string calib_filename;
    std::string roadLink_filename;
public:
    std::vector<std::string> lImages_bw_list; // left image sequence
    std::vector<std::string> rImages_bw_list; // right iamge sequence
    std::vector<std::string> lImages_color_list; // left image sequence
    std::vector<std::string> rImages_color_list; // right iamge sequence
    Dataset(std::string KITTI_DIR);
    sensor_data LoadData(int epoch);
};

#endif // DATASET_H
