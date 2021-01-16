#include "stereomatching.h"

StereoMatching::StereoMatching()
{
}

//StereoMatching::ReadImage
pcl::PointCloud<pcl::RGB>::Ptr StereoMatching::Reader
(std::string lImage_filename  )
{
//    pcl::PCLImage
//    pcl::io::PointCloudImageExtractor a;
//    a.extract()
    cv::Mat image = cv::imread(lImage_filename);
    pcl::PointCloud<pcl::RGB>::Ptr color_cloud
            (new pcl::PointCloud<pcl::RGB> ());
    double* pixel = new double [4];
    memset (pixel, 0, sizeof(double) * 4);
    color_cloud->width = image.cols;
    color_cloud->height = image.rows;
    color_cloud->is_dense = true;
    //color_cloud->points.resize (color_cloud->width * color_cloud->height);
    for (int y = 0; y < image.rows; y++)
    {
        for (int x = 0; x < image.cols; x++)
        {
            if ((x < 0) || (y < 0) || (y > image.rows) || (x > image.cols))
                continue;
            cv::Vec3b intensity = image.at< cv::Vec3b >(y , x);
            uchar blue = intensity.val[0];
            uchar green = intensity.val[1];
            uchar red = intensity.val[2];
            //int rgb , rgba;
            pcl::RGB color;
            color.r = static_cast<uint8_t> (red);
            color.g = static_cast<uint8_t> (green);
            color.b = static_cast<uint8_t> (blue);
            uint32_t rgb = (static_cast<int> (color.r)) << 16 |
                                                  (static_cast<int> (color.g)) << 8 |
                                                  (static_cast<int> (color.b));
            color.rgb = static_cast<float> (rgb);
            color.rgba = static_cast<uint32_t> (rgb);
            color_cloud->points.push_back(color);
        }
    }

    return color_cloud;
}

//StereoMatching::ReadImage
pcl::PointCloud<pcl::PointXYZRGB>::Ptr StereoMatching::Matcher
(pcl::PointCloud<pcl::RGB>::Ptr lImage_PCD  , pcl::PointCloud<pcl::RGB>::Ptr rImage_PCD)
{

    //choice between the two algorithms:
   //pcl::AdaptiveCostSOStereoMatching stereo;
   pcl::BlockBasedStereoMatching stereo;
   stereo.setMaxDisparity(60);
   stereo.setXOffset(0);
   stereo.setRadius(5);
   //only needed for AdaptiveCostSOStereoMatching:
   //stereo.setSmoothWeak(20);
   //stereo.setSmoothStrong(100);
   //stereo.setGammaC(25);
   //stereo.setGammaS(10);
   stereo.setRatioFilter(20);
   stereo.setPeakFilter(0);
   stereo.setLeftRightCheck(true);
   stereo.setLeftRightCheckThreshold(1);
   stereo.setPreProcessing(true);
   stereo.compute(*lImage_PCD, *rImage_PCD);
   stereo.medianFilter(4);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
   stereo.getPointCloud(7.215377e+02,6.095593e+02,1.728540e+02,4.731050e-01 ,
                        out_cloud ,lImage_PCD);
   //stereo.getPointCloud(318.112200, 224.334900, 368.534700, 0.8387445, out_cloud, left_cloud);
   pcl::PointCloud<pcl::RGB>::Ptr vmap( new pcl::PointCloud<pcl::RGB> );
   stereo.getVisualMap(vmap);
   pcl::visualization::ImageViewer iv ("My viewer");
   iv.addRGBImage<pcl::RGB> (vmap);

//    pcl::AdaptiveCostSOStereoMatching stereo;
//    stereo.setMaxDisparity(60);
//    stereo.setXOffset(0);
//    stereo.setRadius(5);
//    stereo.setSmoothWeak(20);
//    stereo.setSmoothStrong(100);
//    stereo.setGammaC(25);
//    stereo.setGammaS(10);
//    stereo.setRatioFilter(20);
//    stereo.setPeakFilter(0);
//    stereo.setLeftRightCheck(true);
//    stereo.setLeftRightCheckThreshold(1);
//    stereo.setPreProcessing(true);
//    stereo.compute(*lImage_PCD, *rImage_PCD);
//    stereo.medianFilter(4);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stereo_cloud
//            (new pcl::PointCloud<pcl::PointXYZRGB>());
////    pcl::PointCloud<pcl::RGB>::Ptr lImage_PCD_pointer =
////            (new pcl::PointCloud<pcl::RGB>(lImage_PCD));
//    stereo.getPointCloud(7.215377e+02,6.095593e+02,1.728540e+02,4.731050e-01 ,
//                         stereo_cloud ,lImage_PCD);
////    pcl::PointCloud<pcl::RGB>::Ptr vmap(new pcl::PointCloud<pcl::RGB> );
////    stereo.getVisualMap(vmap);
////    pcl::visualization::ImageViewer iv("my viewer");
////    iv.addRGBImage <pcl::RGB> (lImage_PCD);
////    iv.spin();
    return out_cloud;
}
