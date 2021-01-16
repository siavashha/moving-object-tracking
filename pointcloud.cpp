#include "pointcloud.h"

PointCloud::PointCloud()
{
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud::PointCloud2imu0
(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , cv::Mat Tvel2imu0)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_cloud
            (new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int32_t i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZRGB p , q;
        p = cloud->points[i];
        cv::Mat pMat(4,1,CV_64F,cv::Scalar(0));
        pMat.at<double>(0,0) = p.x;
        pMat.at<double>(1,0) = p.y;
        pMat.at<double>(2,0) = p.z;
        pMat.at<double>(3,0) = 1.;
        cv::Mat qMat(4,1,CV_64F,cv::Scalar(0.));
        qMat = Tvel2imu0 * pMat;
        q.x = qMat.at<double>(0,0);
        q.y = qMat.at<double>(1,0);
        q.z = qMat.at<double>(2,0);
        q.rgb = p.rgb;
        rotated_cloud->points.push_back(q);
    }
    return rotated_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud::ColorPointCloudFromImage
(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud , cv::Mat Tvel2cam , cv::Mat image)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud
            (new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int32_t i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZI p;
        p = cloud->points[i];
        cv::Mat pMat(4,1,CV_64F,cv::Scalar(0));
        pMat.at<double>(0,0) = p.x;
        pMat.at<double>(1,0) = p.y;
        pMat.at<double>(2,0) = p.z;
        pMat.at<double>(3,0) = 1.;
        cv::Mat qMat(4,1,CV_64F,cv::Scalar(0.));
        qMat = Tvel2cam * pMat;
        int qx = qMat.at<double>(0,0)/qMat.at<double>(2,0);
        int qy = qMat.at<double>(1,0)/qMat.at<double>(2,0);
        if ((qx < 0) || (qy < 0) || (qx > image.cols) || (qy > image.rows))
            continue;
        cv::Vec3b intensity = image.at< cv::Vec3b >(qy , qx);
        uchar blue = intensity.val[0];
        uchar green = intensity.val[1];
        uchar red = intensity.val[2];
        pcl::PointXYZRGB q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        uint32_t rgb = (static_cast<uint32_t>(red) << 16 |
                        static_cast<uint32_t>(green) << 8 | static_cast<uint32_t>(blue));
        q.rgb = *reinterpret_cast<float*>(&rgb);
        //q.z = p.intensity;
        rgbCloud->points.push_back(q);
    }
    return rgbCloud;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloud::CalcNormal(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud)
{
    pcl::NormalEstimation<pcl::PointXYZRGB , pcl::Normal> ne;
    ne.setKSearch(10);
    ne.setInputCloud(cloud);
    //    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZI> ());
    //    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals
            ( new pcl::PointCloud<pcl::Normal>);
    //    ne.setRadiusSearch(0.3);
    ne.compute(*cloud_normals);
    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*cloud_normals, *cloud_normals , indices);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr  cloudN
            (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::concatenateFields(*cloud, *cloud_normals, *cloudN);
    return cloudN;
}

void PointCloud::Ground_Object_disscrimination(
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud ,
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &grourd_cloud ,
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &nonGround_cloud  )
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.50);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
    extract.setNegative (false);
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter (*grourd_cloud);
    extract.setNegative (true);
    extract.filter (*nonGround_cloud);
}

void PointCloud::Smoothing(
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &smoothed_cloud)
{
    pcl::PCA<pcl::PointXYZRGBNormal> pca;
    for (int32_t i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZRGBNormal searchPoint;
        searchPoint = cloud->points[i];
        pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
        kdtree.setInputCloud (cloud);
        // K nearest neighbor search
        int K = 20;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr local
                (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        Eigen::Vector4f center;
        Eigen::Matrix3f cov;
        pcl::computeMeanAndCovarianceMatrix(*cloud , pointIdxNKNSearch , cov , center);
        Eigen::Vector4f plane_parameters;
        float curvature;
        pcl::computePointNormal (*cloud, pointIdxNKNSearch, plane_parameters, curvature);
        std::cout  << " center = " << center << std::endl;
        std::cout  << " covariance = " << cov << std::endl;
        std::cout  << " plane_parameters = " << plane_parameters << std::endl;
        std::cout  << " curvature = " << curvature << std::endl;


//        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//        {

//            std::cout << "K nearest neighbor search at (" << searchPoint.x
//                      << " " << searchPoint.y
//                      << " " << searchPoint.z
//                      << ") with K=" << K << std::endl;

//            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
//            {
//                local->points.push_back(cloud->points[ pointIdxNKNSearch[i] ]);
//                pcl::PointXYZRGB p = cloud->points[ pointIdxNKNSearch[i] ];
//                std::cout << " p = " << p << std::endl;
//            }
//        }

//        pcl::computeCovarianceMatrix
//                (*cloud  , pointIdxNKNSearch , cov);
//        cv::Mat covar (3 , 3 , CV_32F , cov.data());
//        cv::Mat eigenValue ,eigenVector;
//        cv::eigen(covar , eigenValue , eigenVector);
//        std::cout << " i : " << i << std::endl;
//        std::cout << " center : " << center << std::endl;
//        std::cout << " covarianc = " << cov << std::endl;
//        std::cout << "eig_vals = " << eigenValue << std::endl;
//        double l1 = eigenValue.data[0];
//        double l2 = eigenValue.data[1];
//        double l3 = eigenValue.data[2];
//        if (l2 < 0)
//                l2 = 0;
//        if (l3 < 0)
//                l3 = 0;
//        double lambda1 = 1.0 / sqrt( l1 + l2 + l3 + 1 );
//        double lambda2 = 1.0 / ( l1 + l2 + l3 +1 );
//        cv::Mat eigenVector1(3,1,CV_64F);
//        eigenVector1 = eigenVector.row(0).t();
//        cv::Mat eigenVector2(3,1,CV_64F);
//        eigenVector2 = eigenVector.row(1).t();
//        cv::Mat T = lambda1 * eigenVector2 * eigenVector2.t() +
//                 lambda2 * eigenVector1 * eigenVector1.t();
        //std::cout << "T = " << T << std::endl;

        //        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        //        {
        //            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        //                local->points.push_back(cloud->points[ pointIdxNKNSearch[i] ]);
        //            Eigen::Vector4f center;
        //            Eigen::Matrix3f cov;
        //            pcl::computeCovarianceMatrixNormalized
        //                    (cloud , local , center , cov);


        //        }


        // Principal Component Analysis
//        pca.setInputCloud(local);
//        Eigen::Vector3f eig_val = pca.getEigenValues();
//        Eigen::Matrix3f eig_vec = pca.getEigenVectors();

        //std::cout << "eig_val = " << eig_val << std::endl;
        //std::cout << "eig_vec = " << eig_vec << std::endl;


        //            cv::Mat lambda (3,3,CV_64F,cv::Scalar(0));
        //            lambda.at<double>(0,0) = eig_val(0);
        //            lambda.at<double>(1,1) = eig_val(1);
        //            lambda.at<double>(2,2) = eig_val(2);
        //            Eigen::Matrix3f eig;
        //            eig(0,0) = eig_val(0);
        //            eig(1,1) = eig_val(1);
        //            eig(2,2) = eig_val(2);
        ////            std::cout << "I = " << eig_vec.transpose() * eig * eig_vec << std::endl;
        ////            std::cout << "I = " << eig_vec * eig * eig_vec.transpose() << std::endl;


        //            Eigen::MatrixXf eig_coef = pca.getCoefficients();
        //            Eigen::Vector4f meanPCA =  pca.getMean();
        //                    //pca.project();
        //                    //pca.reconstruct();

        //                   // viz->AddPointCloud(*viewer , stereo_cloud );
        ////                    std::cout << "eig_vec = " << eig_vec << std::endl;

        //                    //std::cout << "eig_coef = " << eig_coef << std::endl;
        ////                    std::cout << "meanPCA = " << meanPCA << std::endl;
        ////                std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
        ////                          << " " << cloud->points[ pointIdxNKNSearch[i] ].y
        ////                          << " " << cloud->points[ pointIdxNKNSearch[i] ].z
        ////                          << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        //        }

        ////        //    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZI> ());
        ////        //    ne.setSearchMethod(tree);
   }
}
