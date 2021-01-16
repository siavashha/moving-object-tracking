#include "laserscanner.h"

LaserScanner::LaserScanner()
{

}

void LaserScanner::ReadPCD(std::string filename , pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool filtered)
{
    int32_t num = 1000000;
    float* data = (float*) malloc (num * sizeof (float));
    //pointers
    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pr = data + 3;
    // filename
    FILE* stream;
    stream = fopen(filename.c_str(),"rb");
    num = fread(data , sizeof(float) , num , stream )/ 4;

    for (int32_t i = 0; i < num ; i++)
    {
        // convert points from laser scannar coordinate system
        // into camera coordinate system
        if (filtered )
        {
            //filter angle
            float angle = atan2 (*py,*px) * 180 / 3.14159265359;
            if (std::abs(angle) < 45)
                // filter distance
                if ((*px)*(*px)+(*py)*(*py) < 100. * 100.)
                    //filter height
                    if (*pz > -2.0)
                    {
                        pcl::PointXYZI p;
                        p.x = *px; p.y = *py; p.z = *pz; p.intensity = *pr;
                        cloud->points.push_back(p);
                    }
        }
        else
        {
            pcl::PointXYZI p;
            p.x = *px; p.y = *py; p.z = *pz; p.intensity = *pr;
            cloud->points.push_back(p);
        }
        px+= 4 ; py += 4; pz +=4; pr+=4;
    }
    fclose(stream);
    free(data);
    return;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LaserScanner::PointCloud2imu0 (
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud , cv::Mat Tvel2imu0)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr rotated_cloud
            (new pcl::PointCloud<pcl::PointXYZI>());
    for (int32_t i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZI p , q;
        p = cloud->points[i];
        cv::Mat pMat(4,1,CV_64F,cv::Scalar(0));
        pMat.at<double>(0,0) = p.x;
        pMat.at<double>(1,0) = p.y;
        pMat.at<double>(2,0) = p.z;
        pMat.at<double>(3,0) = 1.;
        cv::Mat qMat(4,1,CV_64F,cv::Scalar(0.));
        qMat = Tvel2imu0 * pMat;
        q.x = qMat.at<double>(0,0);//(p.x * T00 + p.y * T01 + p.z * T02) ;
        q.y = qMat.at<double>(1,0);//(p.x * T10 + p.y * T11 + p.z * T12) ;
        q.z = qMat.at<double>(2,0);//( p.x * T20 + p.y * T21 + p.z * T22 );
        q.intensity = p.intensity;
        rotated_cloud->points.push_back(q);
    }
    return rotated_cloud;
}

void LaserScanner::Ground_Object_disscrimination(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud ,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &grourd_cloud ,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &nonGround_cloud  )
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object


    pcl::SACSegmentationFromNormals<pcl::PointXYZI , pcl::Normal> seg;

    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.3);
    pcl::PointCloud<pcl::Normal>::Ptr normals = CalcNormal(cloud);
    seg.setInputCloud(cloud);
    seg.setInputNormals(normals);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setNegative (false);
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter (*grourd_cloud);

    extract.setNegative (true);
    extract.filter (*nonGround_cloud);

}

std::vector < pcl::PointCloud<pcl::PointXYZI>::Ptr > LaserScanner::Clustering(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.2); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    int j = 0;
    std::vector < pcl::PointCloud<pcl::PointXYZI>::Ptr > clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud->points[*pit]); //*
        clusters.push_back(cloud_cluster);
    }
    return clusters;
}

void LaserScanner::onRoad_offRoad_segmentation(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &road,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &on_road,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &off_road)
{
    pcl::ExtractPolygonalPrismData<pcl::PointXYZI> polygon_extract;
    pcl::PointIndices::Ptr inliers_polygon (new pcl::PointIndices ());
    polygon_extract.setHeightLimits (-10.0, 10.0);
    polygon_extract.setInputPlanarHull (road);
    polygon_extract.setInputCloud (cloud);
    polygon_extract.segment (*inliers_polygon);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setNegative (false);
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_polygon);
    extract.filter (*on_road);

    extract.setNegative (true);
    extract.filter (*off_road);
}

pcl::PointCloud<pcl::Normal>::Ptr LaserScanner::CalcNormal(
        pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud)
{
    pcl::NormalEstimation<pcl::PointXYZI , pcl::Normal> ne;
    ne.setKSearch(10);
    ne.setInputCloud(cloud);
    //    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZI> ());
    //    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals ( new pcl::PointCloud<pcl::Normal>);
    //    ne.setRadiusSearch(0.3);
    ne.compute(*cloud_normals);
    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*cloud_normals, *cloud_normals , indices);
    return cloud_normals;
}

pcl::PointCloud<pcl::SHOT352>::Ptr  LaserScanner::CalcSHOT(
        pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud ,
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normal ,
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keypoints)
{
    pcl::SHOTEstimationOMP <pcl::PointXYZI , pcl::Normal , pcl::SHOT352 > shot;
    shot.setRadiusSearch(1);
    std::vector<int> indices1 , indices2, indices3;
    pcl::removeNaNFromPointCloud(*cloud_keypoints , *cloud_keypoints , indices1);
    shot.setInputCloud(cloud_keypoints);
    pcl::removeNaNNormalsFromPointCloud(*cloud_normal , *cloud_normal , indices2);
    shot.setInputNormals(cloud_normal);
    pcl::removeNaNFromPointCloud(*cloud , *cloud , indices3);
    shot.setSearchSurface (cloud);
    pcl::search::KdTree<pcl::PointXYZI > ::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> );
    shot.setSearchMethod(tree);
    pcl::PointCloud<pcl::SHOT352>::Ptr
            shots (new pcl::PointCloud<pcl::SHOT352>() );
    for (int i = 0 ; i < shots->size() ; i++)
        if  (!pcl_isfinite(shots->at(i).descriptor[0]))
            shots->erase(shots->begin()+ i) ;
    shot.compute(*shots);
    return shots;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr  LaserScanner::CalcFPFH(
        pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud ,
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normal ,
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keypoints)
{
    pcl::FPFHEstimation <pcl::PointXYZI , pcl::Normal , pcl::FPFHSignature33 > fpfh;
    fpfh.setRadiusSearch(1);
    fpfh.setInputCloud(cloud_keypoints);
    fpfh.setInputNormals(cloud_normal);
    fpfh.setSearchSurface (cloud);
    pcl::search::KdTree<pcl::PointXYZI > ::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> );
    fpfh.setSearchMethod(tree);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr
            fpfhs (new pcl::PointCloud<pcl::FPFHSignature33>() );
    fpfh.compute(*fpfhs);
    return fpfhs;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LaserScanner::CalcKeyPoints(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud  )
{
    pcl::PointCloud<int> sampled_indices;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keypoints (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::UniformSampling<pcl::PointXYZI> uniform_sampling;
    uniform_sampling.setInputCloud (cloud);
    uniform_sampling.setRadiusSearch (0.1);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*cloud, sampled_indices.points, *cloud_keypoints);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_keypoints , *cloud_keypoints , indices);
    return cloud_keypoints;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LaserScanner::CalcSIFTKeyPoints(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud  )
{
    // Parameters for sift computation
    const float min_scale = 0.01f;
    const int n_octaves = 3;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.001f;
    // Estimate the sift interest points using Intensity values from RGB values
    pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointWithScale> sift ;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI> ());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud);
    sift.compute(result);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keypoints (new pcl::PointCloud<pcl::PointXYZI>);
    copyPointCloud(result, *cloud_keypoints);
    return cloud_keypoints;
}

pcl::CorrespondencesPtr LaserScanner::CalcSHOTCorrespondance(
        pcl::PointCloud<pcl::SHOT352>::Ptr shot0 ,
        pcl::PointCloud<pcl::SHOT352>::Ptr shot1 )
{
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
    pcl::KdTreeFLANN<pcl::SHOT352> match_search;
    match_search.setInputCloud(shot0);
    for (size_t j = 0; j < shot1->size() ; j++)
    {
        std::vector <int> neigh_indices;
        std::vector <float> neigh_sqr_dists;
        pcl::SHOT352 p = shot1->at (j);
        if (!pcl_isfinite(p.descriptor[0]))//scene_descriptors->at (i).descriptor[0]
        {
            continue;
        }
        int found_neigh = match_search.nearestKSearch( p , 1, neigh_indices , neigh_sqr_dists);
        if (found_neigh == 1 && neigh_sqr_dists[0] < 0.25f)
        {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (j) , neigh_sqr_dists[0]);
            model_scene_corrs->push_back(corr);
        }
    }
    return model_scene_corrs;
}

pcl::CorrespondencesPtr LaserScanner::CalcFPFHCorrespondance(
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr  fpfh0 ,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh1 )
{
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
    pcl::KdTreeFLANN<pcl::FPFHSignature33> match_search;
    match_search.setInputCloud(fpfh0);
    for (size_t j = 0; j < fpfh1->size() ; j++)
    {
        std::vector <int> neigh_indices;
        std::vector <float> neigh_sqr_dists;
        pcl::FPFHSignature33 p = fpfh1->at (j);
        if (!pcl_isfinite(p.histogram[0]))
        {
            continue;
        }
        pcl::FPFHSignature33 corr_fpfh = fpfh1->at(j);
        int found_neigh = match_search.nearestKSearch( corr_fpfh , 1, neigh_indices , neigh_sqr_dists);
        if (found_neigh == 1)// && neigh_sqr_dists[0] < 0.25f)
        {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (j) , neigh_sqr_dists[0]);
            model_scene_corrs->push_back(corr);
        }
    }
    return model_scene_corrs;
}

bool LaserScanner::clusterMatching(std::vector < bool>& visibiity ,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
        std::vector < pcl::PointCloud<pcl::PointXYZI>::Ptr > &clusters,
        std::vector < cv::KalmanFilter >& tracking_clusters ,
        Eigen::Matrix4f &transform_svd )
{
    boost::shared_ptr<pcl::Correspondences> all_correspondences (new pcl::Correspondences);
    Eigen::Vector4f center , center_cluster , diff_centers;
    int size = clusters.size();
    std::vector<double> dist(size);
    pcl::compute3DCentroid(*cloud  , center);
    for (int i = 0 ; i < size ; i++)
    {
        //std::cout << "s1: " << tracking_clusters[i].statePre << std::endl;

        center_cluster(0) = tracking_clusters[i].statePre.at<float>(0,0);
        center_cluster(1) = tracking_clusters[i].statePre.at<float>(1,0);
        center_cluster(2) = tracking_clusters[i].statePre.at<float>(2,0);
        center_cluster(3) = 0;
        //std::cout << "center: " << center << std::endl;
        //pcl::compute3DCentroid(*clusters[i]  , center_cluster );
        diff_centers = center - center_cluster;
        dist[i] = diff_centers.squaredNorm();
    }
    int match_cluster = -1;
    double closest = INFINITY;
    for (int i = 0; i < size ; i ++)
        if (dist[i] < closest)
        {
            closest = dist[i];
            match_cluster = i;
        }
    if (closest > 5.)
    {
        std::cout << " new " << std::endl;

//        if (i==0)
//                int y = 10;
        cv::KalmanFilter* kf = new cv::KalmanFilter(6,3,0,CV_32F);
        Eigen::Vector4f centerN;
        pcl::compute3DCentroid(*cloud  , centerN);
        KF_initialization(*kf , centerN);
        tracking_clusters.push_back(*kf);
        clusters.push_back(cloud);
        return false;
    }
    visibiity[match_cluster] = true;
    Eigen::Vector4f centerM;
    pcl::compute3DCentroid(*cloud , centerM);
    cv::Mat center_M(3,1, CV_32F, cv::Scalar(0.));
    center_M.at<float>(0,0) = centerM(0);
    center_M.at<float>(1,0) = centerM(1);
    center_M.at<float>(2,0) = centerM(2);
    tracking_clusters[match_cluster].correct(center_M);
    //std::cout << "///////////////// " << std::endl;

    //std::cout << "match_cluster " << match_cluster << std::endl;
    ///////////////////
    pcl::PointCloud<pcl::Normal>::Ptr normals0 = CalcNormal(cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr keyPoints0 = CalcKeyPoints(cloud);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr  desc0 = CalcFPFH( cloud , normals0 , keyPoints0) ;
    /////////////////////
    pcl::PointCloud<pcl::Normal>::Ptr normals1 = CalcNormal(clusters[match_cluster]);
    pcl::PointCloud<pcl::PointXYZI>::Ptr keyPoints1 = CalcKeyPoints(clusters[match_cluster]);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr  desc1 = CalcFPFH( clusters[match_cluster] , normals1 , keyPoints1) ;
    /////////////////////
//    std::cout << "normals0 " << normals0->size() << std::endl;
//    std::cout << "keyPoints0 " << keyPoints0->size() << std::endl;
//    std::cout << "desc0 " << desc0->size() << std::endl;
//    std::cout << "normals1 " << normals1->size() << std::endl;
//    std::cout << "keyPoints1 " << keyPoints1->size() << std::endl;
//    std::cout << "desc1 " << desc1->size() << std::endl;
    for (size_t j = 0; j < desc0->size() ; j++)
    {
        pcl::FPFHSignature33 p = desc0->at (j);
        if  (!pcl_isfinite(p.histogram[0]))
        {
            desc0->erase(desc0->begin()+ j) ;
            continue;
        }
    }
    for (size_t j = 0; j < desc1->size() ; j++)
    {
        pcl::FPFHSignature33 p = desc1->at (j);
        if  (!pcl_isfinite(p.histogram[0]))
        {
            desc1->erase(desc1->begin()+ j) ;
            continue;
        }
    }
    /////////////////////

    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    //remove the unknown descriptors
    est.setInputSource (desc0);
    est.setInputTarget (desc1);
    est.determineReciprocalCorrespondences (*all_correspondences);
    double ratio;
    double corr_size = all_correspondences->size();
    (keyPoints0->size() < keyPoints1->size()) ? (ratio = corr_size/keyPoints0->size()) :
                                                (ratio = corr_size/keyPoints1->size());

    //std::cout << "ratio " << ratio << std::endl;


    ////////////////////
    //    double sac_threshold = 0.05;
    //    int sac_max_iterations = 100;
    //    std::vector <int> indices;
    //    boost::shared_ptr<pcl::Correspondences> correspondences_result_rej_sac (new pcl::Correspondences);
    //    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> corrsRejectorSAC;
    //    corrsRejectorSAC.setInputCloud(keyPoints0);
    //    corrsRejectorSAC.setTargetCloud(keyPoints1);
    //    corrsRejectorSAC.setInlierThreshold(sac_threshold);
    //    corrsRejectorSAC.setMaxIterations(sac_max_iterations);
    //    corrsRejectorSAC.setInputCorrespondences(all_correspondences);
    //    corrsRejectorSAC.getCorrespondences(*correspondences_result_rej_sac);
    //    //corrsRejectorSAC.getInliersIndices(indices);

    //    int corr_size = correspondences_result_rej_sac->size();
    //    std::cout << corr_size << " cloud " << corr_size/cloud->size() <<
    //                 " clusters " << corr_size/clusters[match_cluster]->size() << std::endl;
    ////////////////////
    clusters[match_cluster] = cloud;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI> trans_est;
    trans_est.estimateRigidTransformation(*keyPoints0, *keyPoints1, *all_correspondences, transform_svd);
    return true;
}



void LaserScanner::KF_initialization(cv::KalmanFilter &kf, Eigen::Vector4f center)
{
    cv::Mat transition = cv::Mat::eye(6,6,CV_32F);
    transition.at<float>(0,3) = 0.1;
    transition.at<float>(1,4) = 0.1;
    transition.at<float>(2,5) = 0.1;
//    transition.at<float>(3,9) = 0.1;
//    transition.at<float>(4,10) = 0.1;
//    transition.at<float>(5,11) = 0.1;
    kf.transitionMatrix = transition;
    kf.measurementMatrix = cv::Mat::eye(3,6,CV_32F);
    kf.statePost.at<float>(0) = center(0);
    kf.statePost.at<float>(1) = center(1);
    kf.statePost.at<float>(2) = center(2);
    kf.statePost.at<float>(3) = 0.;
    kf.statePost.at<float>(4) = 0.;
    kf.statePost.at<float>(5) = 0.;
//    kf.statePre.at<float>(6) = 0.;
//    kf.statePre.at<float>(7) = 0.;
//    kf.statePre.at<float>(8) = 0.;
//    kf.statePre.at<float>(9) = 0.;
//    kf.statePre.at<float>(10) = 0.;
//    kf.statePre.at<float>(11) = 0.;
    cv::setIdentity(kf.errorCovPost , cv::Scalar::all(0));
    cv::setIdentity(kf.processNoiseCov , cv::Scalar::all( 1 ) );
    cv::setIdentity(kf.measurementNoiseCov , cv::Scalar::all(0.5) );
    //cv::setIdentity(kf.errorCovPre, cv::Scalar::all(.1));
    //cv::setIdentity(kf.errorCovPost, cv::Scalar::all(0.1));
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr LaserScanner::Color_LS
(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
 std::string imgFilename ,
 cv::Mat C_vel2nav, cv::Mat b_vel2nav,cv::Mat C_vel2cam, cv::Mat b_vel2cam)
{
    cv::Mat img = cv::imread(imgFilename,cv::IMREAD_COLOR);
    cv::Mat T_vel2nav(4,4,CV_32F,cv::Scalar(0.));
    T_vel2nav.at<float>(0,0) = C_vel2nav.at<double>(0,0);
    T_vel2nav.at<float>(0,1) = C_vel2nav.at<double>(0,1);
    T_vel2nav.at<float>(0,2) = C_vel2nav.at<double>(0,2);
    T_vel2nav.at<float>(0,3) = b_vel2nav.at<double>(0,0);
    T_vel2nav.at<float>(1,0) = C_vel2nav.at<double>(1,0);
    T_vel2nav.at<float>(1,1) = C_vel2nav.at<double>(1,1);
    T_vel2nav.at<float>(1,2) = C_vel2nav.at<double>(1,2);
    T_vel2nav.at<float>(1,3) = b_vel2nav.at<double>(1,0);
    T_vel2nav.at<float>(2,0) = C_vel2nav.at<double>(2,0);
    T_vel2nav.at<float>(2,1) = C_vel2nav.at<double>(2,1);
    T_vel2nav.at<float>(2,2) = C_vel2nav.at<double>(2,2);
    T_vel2nav.at<float>(2,3) = b_vel2nav.at<double>(2,0);
    T_vel2nav.at<float>(3,3) = 1.;


    cv::Mat T_vel2cam(4,4,CV_32F,cv::Scalar(0.));
    T_vel2cam.at<float>(0,0) = C_vel2cam.at<double>(0,0);
    T_vel2cam.at<float>(0,1) = C_vel2cam.at<double>(0,1);
    T_vel2cam.at<float>(0,2) = C_vel2cam.at<double>(0,2);
    T_vel2cam.at<float>(0,3) = b_vel2cam.at<double>(0,0);
    T_vel2cam.at<float>(1,0) = C_vel2cam.at<double>(1,0);
    T_vel2cam.at<float>(1,1) = C_vel2cam.at<double>(1,1);
    T_vel2cam.at<float>(1,2) = C_vel2cam.at<double>(1,2);
    T_vel2cam.at<float>(1,3) = b_vel2cam.at<double>(1,0);
    T_vel2cam.at<float>(2,0) = C_vel2cam.at<double>(2,0);
    T_vel2cam.at<float>(2,1) = C_vel2cam.at<double>(2,1);
    T_vel2cam.at<float>(2,2) = C_vel2cam.at<double>(2,2);
    T_vel2cam.at<float>(2,3) = b_vel2cam.at<double>(2,0);
    T_vel2cam.at<float>(3,3) = 1.;

    cv::Mat T_nav2cam(4,4,CV_32F,cv::Scalar(0.));
    T_nav2cam = T_vel2cam * T_vel2nav.t();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_PCD(new pcl::PointCloud<pcl::PointXYZRGB>());
    cv::Mat R00_rect(4,4,CV_32F,cv::Scalar(0));
    R00_rect.at<float>(0,0) = 9.998817e-01;
    R00_rect.at<float>(0,1) = 1.511453e-02;
    R00_rect.at<float>(0,2) = -2.841595e-03;
    R00_rect.at<float>(0,3) = 0.;


    R00_rect.at<float>(1,0) = -1.511724e-02;
    R00_rect.at<float>(1,1) = 9.998853e-01;
    R00_rect.at<float>(1,2) = -9.338510e-04 ;
    R00_rect.at<float>(1,3) = 0.;

    R00_rect.at<float>(2,0) =  2.827154e-03 ;
    R00_rect.at<float>(2,1) =  9.766976e-04 ;
    R00_rect.at<float>(2,2) = 9.999955e-01;
    R00_rect.at<float>(2,3) = 0.;
    R00_rect.at<float>(3,3) = 1.;
    cv::Mat P00_rect(3,4,CV_32F,cv::Scalar(0));
    //R_rect_02: 9.998817e-01 1.511453e-02 -2.841595e-03 -1.511724e-02 9.998853e-01 -9.338510e-04 2.827154e-03 9.766976e-04 9.999955e-01
    //P_rect_02: 7.215377e+02 0.000000e+00 6.095593e+02 4.485728e+01
    //    0.000000e+00 7.215377e+02  1.728540e+02 2.163791e-01
    //    0.000000e+00 0.000000e+00 1.000000e+00 2.745884e-03
    P00_rect.at<float>(0,0) =7.215377e+02;
    P00_rect.at<float>(0,1) =0.000000e+00;
    P00_rect.at<float>(0,2) =6.095593e+02;
    P00_rect.at<float>(0,3) =4.485728e+01;
    P00_rect.at<float>(1,0) =0.000000e+00;
    P00_rect.at<float>(1,1) =7.215377e+02;
    P00_rect.at<float>(1,2) =1.728540e+02;
    P00_rect.at<float>(1,3) =2.163791e-01;
    P00_rect.at<float>(2,0) =0.000000e+00;
    P00_rect.at<float>(2,1) =0.000000e+00;
    P00_rect.at<float>(2,2) =1.000000e+00;
    P00_rect.at<float>(2,3) =2.745884e-03;
    // compose T matrix

    // compose T matrix
    cv::Mat PR0 = P00_rect * R00_rect  ;
    cv::Mat PR = PR0 * T_nav2cam;
    float P00 = PR.at<float>(0,0);
    float P01 = PR.at<float>(0,1);
    float P02 = PR.at<float>(0,2);
    float P03 = PR.at<float>(0,3);
    float P10 = PR.at<float>(1,0);
    float P11 = PR.at<float>(1,1);
    float P12 = PR.at<float>(1,2);
    float P13 = PR.at<float>(1,3);
    float P20 = PR.at<float>(2,0);
    float P21 = PR.at<float>(2,1);
    float P22 = PR.at<float>(2,2);
    float P23 = PR.at<float>(2,3);

    for (int32_t i = 0 ; i < cloud->points.size() ; i++)
    {
        pcl::PointXYZI p;
        pcl::PointXYZRGB q , r;
        p = cloud->points[i];
        float r0 = p.x * P00 + p.y * P01 + p.z * P02 + P03;
        float r1 = p.x * P10 + p.y * P11 + p.z * P12 + P13;
        float r2 = p.x * P20 + p.y * P21 + p.z * P22 + P23;
        //cloud->points[i].x = r0 / r2; cloud->points[i].y = r1 / r2; cloud->points[i].z = r2;
        q.x = (float) (r0 / r2);
        q.y = (float) (r1 / r2);
        q.z = (float) (r2);
        int qx = static_cast<int>(q.x);//+ img.cols/2
        int qy = static_cast<int>(q.y);//+ img.rows/2

        r.x = p.x;
        r.y = p.y;
        r.z = p.z;
        if ((qx < 0) || (qy < 0) || (qx > img.cols) || (qy > img.rows))
            continue;
        cv::Vec3b intensity = img.at< cv::Vec3b >(qy , qx);
        uchar blue = intensity.val[0];
        uchar green = intensity.val[1];
        uchar red = intensity.val[2];

        uint32_t rgb = (static_cast<uint32_t>(red) << 16 |
                        static_cast<uint32_t>(green) << 8 | static_cast<uint32_t>(blue));
        r.rgb = *reinterpret_cast<float*>(&rgb);
        proj_PCD->points.push_back(r);
    }
    return proj_PCD;
}


cv::Mat LaserScanner::imageProjection(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
         std::string imgFilename ,
         cv::Mat C_vel2nav, cv::Mat b_vel2nav,cv::Mat C_vel2cam, cv::Mat b_vel2cam)
{
    cv::Mat img = cv::imread(imgFilename,cv::IMREAD_COLOR);
    cv::Mat T_vel2nav(4,4,CV_32F,cv::Scalar(0.));
    T_vel2nav.at<float>(0,0) = C_vel2nav.at<double>(0,0);
    T_vel2nav.at<float>(0,1) = C_vel2nav.at<double>(0,1);
    T_vel2nav.at<float>(0,2) = C_vel2nav.at<double>(0,2);
    T_vel2nav.at<float>(0,3) = b_vel2nav.at<double>(0,0);
    T_vel2nav.at<float>(1,0) = C_vel2nav.at<double>(1,0);
    T_vel2nav.at<float>(1,1) = C_vel2nav.at<double>(1,1);
    T_vel2nav.at<float>(1,2) = C_vel2nav.at<double>(1,2);
    T_vel2nav.at<float>(1,3) = b_vel2nav.at<double>(1,0);
    T_vel2nav.at<float>(2,0) = C_vel2nav.at<double>(2,0);
    T_vel2nav.at<float>(2,1) = C_vel2nav.at<double>(2,1);
    T_vel2nav.at<float>(2,2) = C_vel2nav.at<double>(2,2);
    T_vel2nav.at<float>(2,3) = b_vel2nav.at<double>(2,0);
    T_vel2nav.at<float>(3,3) = 1.;


    cv::Mat T_vel2cam(4,4,CV_32F,cv::Scalar(0.));
    T_vel2cam.at<float>(0,0) = C_vel2cam.at<double>(0,0);
    T_vel2cam.at<float>(0,1) = C_vel2cam.at<double>(0,1);
    T_vel2cam.at<float>(0,2) = C_vel2cam.at<double>(0,2);
    T_vel2cam.at<float>(0,3) = b_vel2cam.at<double>(0,0);
    T_vel2cam.at<float>(1,0) = C_vel2cam.at<double>(1,0);
    T_vel2cam.at<float>(1,1) = C_vel2cam.at<double>(1,1);
    T_vel2cam.at<float>(1,2) = C_vel2cam.at<double>(1,2);
    T_vel2cam.at<float>(1,3) = b_vel2cam.at<double>(1,0);
    T_vel2cam.at<float>(2,0) = C_vel2cam.at<double>(2,0);
    T_vel2cam.at<float>(2,1) = C_vel2cam.at<double>(2,1);
    T_vel2cam.at<float>(2,2) = C_vel2cam.at<double>(2,2);
    T_vel2cam.at<float>(2,3) = b_vel2cam.at<double>(2,0);
    T_vel2cam.at<float>(3,3) = 1.;

    cv::Mat T_nav2cam(4,4,CV_32F,cv::Scalar(0.));
    T_nav2cam = T_vel2cam * T_vel2nav.t();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_PCD(new pcl::PointCloud<pcl::PointXYZRGB>());
    cv::Mat R00_rect(4,4,CV_32F,cv::Scalar(0));
    R00_rect.at<float>(0,0) = 9.998817e-01;
    R00_rect.at<float>(0,1) = 1.511453e-02;
    R00_rect.at<float>(0,2) = -2.841595e-03;
    R00_rect.at<float>(0,3) = 0.;


    R00_rect.at<float>(1,0) = -1.511724e-02;
    R00_rect.at<float>(1,1) = 9.998853e-01;
    R00_rect.at<float>(1,2) = -9.338510e-04 ;
    R00_rect.at<float>(1,3) = 0.;

    R00_rect.at<float>(2,0) =  2.827154e-03 ;
    R00_rect.at<float>(2,1) =  9.766976e-04 ;
    R00_rect.at<float>(2,2) = 9.999955e-01;
    R00_rect.at<float>(2,3) = 0.;
    R00_rect.at<float>(3,3) = 1.;
    cv::Mat P00_rect(3,4,CV_32F,cv::Scalar(0));
    //R_rect_02: 9.998817e-01 1.511453e-02 -2.841595e-03 -1.511724e-02 9.998853e-01 -9.338510e-04 2.827154e-03 9.766976e-04 9.999955e-01
    //P_rect_02: 7.215377e+02 0.000000e+00 6.095593e+02 4.485728e+01
    //    0.000000e+00 7.215377e+02  1.728540e+02 2.163791e-01
    //    0.000000e+00 0.000000e+00 1.000000e+00 2.745884e-03
    P00_rect.at<float>(0,0) =7.215377e+02;
    P00_rect.at<float>(0,1) =0.000000e+00;
    P00_rect.at<float>(0,2) =6.095593e+02;
    P00_rect.at<float>(0,3) =4.485728e+01;
    P00_rect.at<float>(1,0) =0.000000e+00;
    P00_rect.at<float>(1,1) =7.215377e+02;
    P00_rect.at<float>(1,2) =1.728540e+02;
    P00_rect.at<float>(1,3) =2.163791e-01;
    P00_rect.at<float>(2,0) =0.000000e+00;
    P00_rect.at<float>(2,1) =0.000000e+00;
    P00_rect.at<float>(2,2) =1.000000e+00;
    P00_rect.at<float>(2,3) =2.745884e-03;
    // compose T matrix

    // compose T matrix
    cv::Mat PR0 = P00_rect * R00_rect  ;
    cv::Mat PR = PR0 * T_nav2cam;
    float P00 = PR.at<float>(0,0);
    float P01 = PR.at<float>(0,1);
    float P02 = PR.at<float>(0,2);
    float P03 = PR.at<float>(0,3);
    float P10 = PR.at<float>(1,0);
    float P11 = PR.at<float>(1,1);
    float P12 = PR.at<float>(1,2);
    float P13 = PR.at<float>(1,3);
    float P20 = PR.at<float>(2,0);
    float P21 = PR.at<float>(2,1);
    float P22 = PR.at<float>(2,2);
    float P23 = PR.at<float>(2,3);
    cv::Mat imgTemp = img.clone();

    for (int32_t i = 0 ; i < cloud->points.size() ; i++)
    {
        cv::Point2f point ;
        pcl::PointXYZI p , q;
        p = cloud->points[i];
        float r0 = p.x * P00 + p.y * P01 + p.z * P02 + P03;
        float r1 = p.x * P10 + p.y * P11 + p.z * P12 + P13;
        float r2 = p.x * P20 + p.y * P21 + p.z * P22 + P23;
        //cloud->points[i].x = r0 / r2; cloud->points[i].y = r1 / r2; cloud->points[i].z = r2;
        q.x = (float) (r0 / r2);
        q.y = (float) (r1 / r2);
        q.z = (float) (r2);
        point.x = static_cast<int>(q.x);//+ img.cols/2
        point.y = static_cast<int>(q.y);//+ img.rows/2
        std::cout << " point " << point << std::endl;
        cv::circle(imgTemp ,point ,1,cv::Scalar(0, 0, 0),2,8);
    }
    //color = GroundColorMix(((float)(p.z) * 360. / maxDepth), minDepth, maxDepth);

    cv::imshow("projectedLS",imgTemp);
    cv::waitKey(0);
    return imgTemp;
}

