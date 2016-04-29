#ifndef GETSOURCECLOUD_H
#define GETSOURCECLOUD_H

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

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class GetSourceCloud
{
public:
    GetSourceCloud();

    void getParam(double fx, double fy, double cx, double cy);
    void getCloud(cv::Mat color, cv::Mat depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr  &cloudSource, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr  &cloudNC);

private:
    double fx_;
    double fy_;
    double cx_;
    double cy_;
};

#endif // GETSOURCECLOUD_H
