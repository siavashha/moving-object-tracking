#include "visualizer.h"

Visualizer::Visualizer(pcl::visualization::PCLVisualizer& viewer)
{
    //viewer.setBackgroundColor (255, 255, 255);
    //0.159191,159.191/4.434,-6.00264,-0.551972/21.5358,-6.73144,74.6503/0.00139942,0.999955,0.00937244/0.523599/640,747/66,52
  //  viewer.setCameraPosition(13.1726,-14.8732,92.1854,6.02365,-12.6087,-0.834961,-0.0037116,0.99969,0.0246217);
                //21.5358,-6.73144,74.6503 , 4.434,-6.00264,-0.551972 , 0.00139942,0.999955,0.00937244);
//    viewer.setCameraPosition(-4.78864,6.42148,117.489,
//                             -23.8944,28.1957,-5.91881
//                             ,0.0196266,0.985114,0.170776);

//    viewer.setCameraPosition(-34.5519,15.4022,272.585,
//                             -75.5068,62.0771,8.04981
//                             ,0.0196266,0.985114,0.170776);
   // -75.5068,62.0771,8.04981/-34.5519,15.4022,272.585/0.0196266,0.985114,0.170776


  //  237.821,296.22////0.523599/396,480/310,52


/*viewer.setCameraPosition(
    23.3439,-19.2128,27.5049
            ,21.066,-18.4913,-2.13423
            ,-0.0037116,0.99969,0.0246217)*/;


//    16.9745,-13.4844,-1.69789/24.1234,-15.7489,91.3225/-0.0037116,0.99969,0.0246217
//    0.00522511,5.22511/0.0427789,-0.185814,0.0496169/0.0497412,-0.196849,-0.0978747/-0.0956887,-0.992963,0.0697719/0.523599/631,491/1650,152
//            m_pclVisualizer->setBackgroundColor(0, 0, 0);
//            m_pclVisualizer->setCameraPosition(0.0497412, -0.196849, -0.0978747, 0.0427789, -0.185814, 0.0496169, -0.0956887, -0.992963, 0.0697719);
//            m_pclVisualizer->setCameraFieldOfView(0.523599);
//            m_pclVisualizer->setCameraClipDistances(0.00522511, 5.22511);
//            m_pclVisualizer->setPosition(1650, 152);
//            m_pclVisualizer->setSize(631, 491);



    //viewer.createViewPort(0.0, 0.0, 0.0, 1.0, v1);
    viewer.addCoordinateSystem();
    //int v1=0;
    //viewer.setCameraPosition (0, 0, 230, 0, 0, 0,0,1,0);
    //viewer.setCameraParameters(0.159191,159.191/4.434,-6.00264,-0.551972/21.5358,-6.73144,74.6503/0.00139942,0.999955,0.00937244/0.523599/640,747/66,52);
    viewer.setSize (640,480);  // Visualiser window size
    //viewer.updateCamera();

    object_counter=0;
}

Visualizer::Visualizer(pcl::visualization::PCLVisualizer& viewer , cv::Mat orientation)
{
    //viewer.createViewPort(0.0, 0.0, 0.0, 1.0, v1);
    viewer.setBackgroundColor (255, 255, 255);
    Eigen::Affine3f t;
    t(0,0)=orientation.at<double>(0,0);
    t(1,0)=orientation.at<double>(1,0);
    t(2,0)=orientation.at<double>(2,0);
    t(3,0)=0.;
    t(0,1)=orientation.at<double>(0,1);
    t(1,1)=orientation.at<double>(1,1);
    t(2,1)=orientation.at<double>(2,1);
    t(3,1)=0.;
    t(0,2)=orientation.at<double>(0,2);
    t(1,2)=orientation.at<double>(1,2);
    t(2,2)=orientation.at<double>(2,2);
    t(3,2)=0.;
    t(0,3)=0.;
    t(1,3)=0.;
    t(2,3)=0.;
    t(3,3)=1.;

    viewer.addCoordinateSystem(1.0 , t);
}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    object_counter++;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
            handler (cloud, 0, 0, 255);
    viewer.addPointCloud(cloud, handler , "cloud"+object_counter);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud"+object_counter);
}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud ,
                               int epoch)
{
    object_counter++;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
            handler (cloud, epoch * 20, epoch * 20, 255);
    viewer.addPointCloud(cloud, handler , "cloud"+object_counter);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud"+object_counter);
    Eigen::Vector4f center;
    //pcl::compute3DCentroid(*cloud  , center );
    center(0)=0;
    center(1) =0;
    center(2)=0;
    std::cout << "center: " << center << std::endl;
    pcl::PointXYZI p;
    p.x = center(0); p.y = center(1); p.z = center(2); p.intensity = 100;
    std::string object;
    object = "id" + boost::lexical_cast<std::string>(epoch) +
            "\n" + "x:" + boost::lexical_cast<std::string>(p.x) +
            "\n" + "y:" + boost::lexical_cast<std::string>(p.y) +
            "\n" + "z:" + boost::lexical_cast<std::string>(p.z);

    viewer.addText3D<pcl::PointXYZI>
            (object, p, 0.1 ,0,0,0);
}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    object_counter++;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
            handler (cloud);
    viewer.addPointCloud(cloud, handler , "cloudRGB"+object_counter);
    viewer.setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudRGB"+object_counter);

}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud ,
                               cv::Point3f color )
{
        int r =color.x, g=color.y, b=color.z ;
        object_counter++;
//        double Pr=0.299;
//        double Pg=0.587;
//        double Pb=0.114;
//        double  P =std::sqrt(r*r*Pr + g*g*Pg + b*b*Pb ) ;
//        double change = epoch / 10;
//        r=P+(r-P)*change;
//        g=P+(g-P)*change;
//        b=P+(b-P)*change;
        std::string str = "cluster"+boost::lexical_cast<std::string>(object_counter);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
                handler(cloud, r, g, b);
        viewer.addPointCloud(cloud, handler , str);
        viewer.setPointCloudRenderingProperties (
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                    5, str);
}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ,
                               std::string color)
{
    int r =0, g=0, b=0 ;
    if (color.compare("red")== 0)
    {r=255;g=0;b=0;}
    else if (color.compare("silver")== 0)
    {r=192;g=192;b=192;}
    else if (color.compare("teal")== 0)
    {r=0;g=128;b=128;}
    else if (color.compare("lightblue")==0)
    {r=173;g=216;b=230;}


    object_counter++;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>
            handler(cloud, r, g, b);
    std::string str = "cloud"+boost::lexical_cast<std::string>(object_counter);
    viewer.addPointCloud(cloud, handler , str);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, str);

}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               cv::Mat b)
{
    object_counter++;
    pcl::PointXYZI p , q;
    p.x = b.at<double>(0,0);
    p.y = b.at<double>(1,0);
    p.z = b.at<double>(2,0);
    p.intensity = 100;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    q.intensity = 100;
    std::string strSphere = "sphere"+boost::lexical_cast<std::string>(object_counter);
    viewer.addSphere (p, 0.2, 0.5, 0.5, 0.0, strSphere);
    std::string strArrow = "arrow_car"+boost::lexical_cast<std::string>(object_counter);
    //viewer.addArrow<pcl::PointXYZI, pcl::PointXYZI>(p,q,0,1,0,false,strArrow);
}

//void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
//                               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud0,
//                               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
//                               boost::shared_ptr<pcl::Correspondences> corr)
//{
//    object_counter++;
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
//            handler (cloud, 0, 0, 255);
//    viewer.addPointCloud(cloud, handler , "cloud"+object_counter);
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud"+object_counter);
//}


void Visualizer::ShowCoordinateSystem(pcl::visualization::PCLVisualizer& viewer,
                                      std::vector<cv::Mat> ENU)
{
    pcl::PointXYZI pO ,pN , pE , pU;

    pO.x = 0; pO.y = 0; pO.z = 0; pO.intensity = 100;

    cv::Mat East;
    East = ENU[0];
    pE.x = East.at<double>(0,0);
    pE.y = East.at<double>(1,0);
    pE.z = East.at<double>(2,0);
    pE.intensity = 100;

    cv::Mat North;
    North = ENU[1];
    pN.x = North.at<double>(0,0);
    pN.y = North.at<double>(1,0);
    pN.z = North.at<double>(2,0);
    pN.intensity = 100;

    cv::Mat Up;
    Up = ENU[2];
    pU.x = Up.at<double>(0,0);
    pU.y = Up.at<double>(1,0);
    pU.z = Up.at<double>(2,0);
    pU.intensity = 100;

    viewer.addArrow<pcl::PointXYZI, pcl::PointXYZI>
            (pE,pO, 1,1,1,false,"arrow1");
    viewer.addText3D<pcl::PointXYZI>
            ("E", pE , 1 ,1,1,1);

    viewer.addArrow<pcl::PointXYZI, pcl::PointXYZI>
            (pN,pO, 1,1,1,false,"arrow2");
    viewer.addText3D<pcl::PointXYZI>
            ("N", pN, 1 ,1,1,1);

    viewer.addArrow<pcl::PointXYZI, pcl::PointXYZI>
            (pU,pO, 1,1,1,false,"arrow3");
    viewer.addText3D<pcl::PointXYZI>
            ("U", pU , 1 ,1,1,1);

}

void Visualizer::ShowRoadLinks(pcl::visualization::PCLVisualizer& viewer,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr roadlinkcloud)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            RoadLinkhandler (roadlinkcloud, 0, 255, 0);
    viewer.addPointCloud(roadlinkcloud, RoadLinkhandler , "RoadLinks");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "RoadLinks");
    for (int  i = 0 ; i < roadlinkcloud->size() - 1; i=i+1)
    {
        std::string ss_line;
        ss_line = "link" + boost::lexical_cast<std::string>(i) + "_" +
                           boost::lexical_cast<std::string>(i+1) ;
        viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>
                (roadlinkcloud->points[i], roadlinkcloud->points[i+1] ,
                 0, 255, 0, ss_line);
//        std::string roadPoint;
//        roadPoint = "x:" +  boost::lexical_cast<std::string>( roadlinkcloud->points[i].x) +
//                "\n" + "y:" + boost::lexical_cast<std::string>(roadlinkcloud->points[i].y) +
//                "\n" + "z:" + boost::lexical_cast<std::string>(roadlinkcloud->points[i].z);

//                viewer.addText3D<pcl::PointXYZI>
//                (roadPoint, roadlinkcloud->points[i] , 0.1 ,0,0,0);
    }
}

/**
    * Computes the color gradiant
    * color: the output vector
    * x: the gradiant (beetween 0 and 360)
    * min and max: variation of the RGB channels (Move3D 0 -> 1)
    */
cv::Point3f Visualizer::GroundColorMix( float x, float min, float max)
{
    /*
   * Red = 0
   * Green = 1
   * Blue = 2
   */
    cv::Point3f color;
    float posSlope = (max-min)/60;
    float negSlope = (min-max)/60;

    if( x < 60 )
    {
        color.x = (int) (max);
        color.y = (int) (posSlope*x+min);
        color.z = (int) (min);
    }
    else if ( x < 120 )
    {
        color.x = (int) (negSlope*x+2*max+min);
        color.y = (int) (max);
        color.z = (int) (min);
    }
    else if ( x < 180  )
    {
        color.x  = (int) (min);
        color.y = (int) (max);
        color.z = (int) (posSlope*x-2*max+min);
    }
    else if ( x < 240  )
    {
        color.x = (int) (min);
        color.y = (int) (negSlope*x+4*max+min);
        color.z = (int) (max);
    }
    else if ( x < 300  )
    {
        color.x = (int) (posSlope*x-4*max+min);
        color.y = (int) (min);
        color.z = (int) (max);
    }
    else
    {
        color.x = (int) (max);
        color.y = (int) (min);
        color.z = (int) (negSlope*x+6*max);
    }
    return color;
}

void Visualizer::FindMinMaxPCD(float& max , float& min, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_PCD)
{
    max = 0.;
    min = 1000.;
    for (int32_t i = 0 ; i < proj_PCD->points.size() ; i++)
    {
        pcl::PointXYZRGB p = proj_PCD->points[i];
        if (p.z > max)
            max = p.z;
        if (p.z < min)
            min = p.z;
    }
}

void Visualizer::ProjectedPCD_Vis(std::string imgFilename , const pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_PCD)
{
    cv::Mat img = cv::imread(imgFilename,cv::IMREAD_COLOR);
    float maxDepth , minDepth;
    FindMinMaxPCD(maxDepth , minDepth, proj_PCD);
    std::cout << " max " << maxDepth << " min " << minDepth << std::endl;
    cv::Point3f color;
    for (int i = 0 ; i < proj_PCD->size() ; i++)
    {
        cv::Point2f q;
        pcl::PointXYZRGB p = proj_PCD->points[i];

        q.x = (float) (p.x) ;
        q.y = (float) (p.y) ;
        color = GroundColorMix(((float)(p.z) * 360. / maxDepth), minDepth, maxDepth);
        cv::circle(img ,q ,1,cv::Scalar(color.z, color.y, color.x),2,8);
    }
    cv::imshow("projectedLS",img);
    cv::waitKey(0);

}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ,
                               pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    object_counter++;

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals);//, 10, 0.05, "normals");
   // viewer.addCoordinateSystem (1.0);
  //  viewer.initCameraParameters ();


//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
//            handler (cloud);
//    viewer.addPointCloud(cloud, handler , "cloudRGBNormals"+object_counter);
//    viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "cloudRGBNormals"+object_counter);
//    viewer.setPointCloudRenderingProperties
//            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudRGBNormals"+object_counter);
}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    object_counter++;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>
            handler (cloud);
    viewer.addPointCloud(cloud, handler , "cloudRGBNormal"+object_counter);
    viewer.setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudRGBNormal"+object_counter);


//    object_counter++;
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>
//            handler (cloud);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> color2 (cloud, 0, 0, 255);
//    viewer.addPointCloud (cloud, color2, "cloudRGBNormal"+object_counter);
//    viewer.addPointCloud(cloud, handler , "cloudRGBNormal"+object_counter);
////    viewer.addPointCloud (cloud , "normals" ,);
////    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "normals");

////    viewer.setPointCloudRenderingProperties
////            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudRGBNormal"+object_counter);
//    //pcl::visualization::PCLVisualizerInteractorStyle::OnKeyDown();

//    //    viewer.setPointCloudRenderingProperties
////            (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloudRGBNormal"+object_counter);
}
