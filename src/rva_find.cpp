//STL
#include <iostream>
#include <math.h>
#include <vector>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//This project
#include "findplane.h"
#include "getsourcecloud.h"
#include "utilities.h"

using namespace std;

/*globel parameters*/
pcl::visualization::PCLVisualizer viewer ("Visualization");
pcl::console::TicToc tt;

string fileName_ = "33";
double fx_ = 538.77;
double fy_ = 540.23;
double cx_ = 314.76;
double cy_ = 239.95;

float rx_ = 112.0 /180 * M_PI; //depression angle:22 + 90
float ry_ = 1.0 /180 * M_PI;
Eigen::Matrix4f transform_inv_ = Eigen::Matrix4f::Identity();

float gh_ = 1.20;

int visualizeCounter = 0;


void visualizeThings(pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_hull, std::string pname, std::string lname)
{
    double cr, cg, cb;
    pcl::visualization::getRandomColors(cr,cg,cb);

    viewer.addPolygon<pcl::PointXYZ>(cloud_hull,cr, cg, cb, pname);
    viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,pname);
    viewer.addText3D(lname, cloud_hull->points[0], 0.02, 0.5, 0.5, 0.9, lname);
}


int main(int argc, char** argv)
{
    viewer.setPosition(500, 250); // Setting visualizer window position

    if (argc == 2)//if there is any arg, it starts form 1
        {
            std::string arg(argv[1]);
            fileName_ = arg;
        }

    cv::Mat colorImg = cv::imread(fileName_+ ".jpg", 1);
    cv::Mat depthImg = cv::imread(fileName_ + ".png", CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH);
    if (colorImg.empty() || depthImg.empty())
        {
            cerr << "No image found." << endl;
            return -1;
        }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSource (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudNC (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudT (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());

    GetSourceCloud GSC;
    GSC.getParam(fx_, fy_, cx_, cy_);
    GSC.getCloud(colorImg, depthImg, cloudSource, cloudNC);

//    string name = "cloud";
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudSource);
//    viewer.addPointCloud<pcl::PointXYZRGB> (cloudSource, rgb, name);
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
//    viewer.addCoordinateSystem(0.2);

//    Utilities::writeCloud(cloudNC,"./cloud_with_nor.pcd");

    Utilities::rotateCloudXY(cloudNC, cloudT, rx_, ry_, transform_inv_);

    string name_t = "cloud_t";
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(cloudT);
    viewer.addPointCloud<pcl::PointXYZRGBNormal> (cloudT, rgb, name_t);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name_t);
    viewer.addCoordinateSystem(0.2);

//    Utilities::writeCloud(cloudT,"./cloud_t.pcd");

    FindPlane FP;
    FP.getParameters(gh_);
    FP.findPlaneInCloud(cloudT);
    for (size_t i = 0; i < FP.plane_hull.size(); i++)
        {
            string p_name, label_name;
            Utilities::generateName(i,"ps_","",p_name);
            Utilities::generateName(i,"surface_","",label_name);
            visualizeThings(FP.plane_hull[i], p_name, label_name);
        }
    for (size_t i = 0; i < FP.ground_hull.size(); i++)
        {
            string p_name, label_name;
            Utilities::generateName(i,"pg_","",p_name);
            Utilities::generateName(i,"ground_","",label_name);
            visualizeThings(FP.ground_hull[i], p_name, label_name);
        }



    while (!viewer.wasStopped ()) { // Display the visualizer until 'q' key is pressed
            viewer.spinOnce ();
        }
}
