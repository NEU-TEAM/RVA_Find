#include "utilities.h"

using namespace std;

Utilities::Utilities()
{
}

void Utilities::getParameters(std::string argString, std::vector <std::string> &outString, std::vector <float> &outNum)
{
//  for (size_t i = 0; i < (int)argString.size(); i++)
//    {
//      if (argString[i])
//    }
}

void Utilities::generateName(int count, std::string pref, std::string surf, std::string &name)
{
  std::ostringstream ost;
  ost << count;
  std::string temp(ost.str());
  name = pref + temp + surf;
}

void Utilities::pointTypeTransfer(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{
  cloud_out->resize(cloud_in->size());
  for (size_t i = 0; i < cloud_in->points.size(); i++) {
      cloud_out->points[i].x = cloud_in->points[i].x;
      cloud_out->points[i].y = cloud_in->points[i].y;
      cloud_out->points[i].z = cloud_in->points[i].z;
    }
}

void Utilities::cutCloud(pcl::ModelCoefficients::Ptr coeff_in, double th_distance,
                         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{
    std::vector <int> inliers_cut;
    Eigen::Vector4f coeffs(coeff_in->values[0],coeff_in->values[1],coeff_in->values[2],coeff_in->values[3]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSourceFiltered_t (new pcl::PointCloud<pcl::PointXYZ> ());
    pointTypeTransfer(cloud_in, cloudSourceFiltered_t);
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> scmp(cloudSourceFiltered_t);
    scmp.selectWithinDistance (coeffs, th_distance, inliers_cut);
    scmp.projectPoints(inliers_cut, coeffs, *cloud_out, false);
}

void Utilities::ecExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices,
                             double th_cluster, int minsize, int maxsize)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (th_cluster);
    ec.setMinClusterSize (minsize);//should be small, let area judge
    ec.setMaxClusterSize (maxsize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_in);
    ec.extract (cluster_indices);
}

void Utilities::rotateCloudXY(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_out,
	float rx, float ry, Eigen::Matrix4f &transform_inv)
{
	Eigen::Matrix4f transform_x = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();

	//the math function cos etc. operate angle in radius
	transform_x(1,1) = cos(rx);
	transform_x(2,1) = -sin(rx);
	transform_x(1,2) = sin(rx);
	transform_x(2,2) = cos(rx);
	std::cout << "trans x: "<< transform_x << std::endl;

	transform_y(0,0) = cos(ry);
	transform_y(0,2) = -sin(ry);
	transform_y(2,0) = sin(ry);
	transform_y(2,2) = cos(ry);
	std::cout << "trans y: "<< transform_y << std::endl;

	transform_ = transform_y * transform_x;
	std::cout << "total trans: "<< transform_ << std::endl;
	transform_inv = transform_.inverse();
	std::cout << "trans_inv: "<< transform_inv << std::endl;

	// Executing the transformation
	pcl::transformPointCloudWithNormals(*cloud_in, *cloud_out, transform_);
}



void Utilities::writeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string name)
{
	pcl::PCDWriter writer;
	if (cloud->points.empty ())
		std::cerr << "Can't find component." << std::endl;
	else
		{
			writer.write (name, *cloud, true);
		}
}

void Utilities::writeCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, std::string name)
{
	pcl::PCDWriter writer;
	if (cloud->points.empty ())
		std::cerr << "Can't find component." << std::endl;
	else
		{
			writer.write (name, *cloud, true);
		}
}
