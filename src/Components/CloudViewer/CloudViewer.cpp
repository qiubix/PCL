/*!
 * \file
 * \brief
 * \author Maciej Stefańczyk [maciek.slon@gmail.com]
 */

#include <memory>
#include <string>

#include "CloudViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace CloudViewer {

CloudViewer::CloudViewer(const std::string & name) :
		Base::Component(name),
    prop_window_name("window_name", std::string("3D PC Viewer"))
{
  registerProperty(prop_window_name);
}

CloudViewer::~CloudViewer() {
}

void CloudViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_normals", &in_cloud_normals);

	// Register handlers
	h_on_cloud_xyz.setup(boost::bind(&CloudViewer::on_cloud_xyz, this));
	registerHandler("on_cloud_xyz", &h_on_cloud_xyz);
	addDependency("on_cloud_xyz", &in_cloud_xyz);
	h_on_cloud_xyzrgb.setup(boost::bind(&CloudViewer::on_cloud_xyzrgb, this));
	registerHandler("on_cloud_xyzrgb", &h_on_cloud_xyzrgb);
	addDependency("on_cloud_xyzrgb", &in_cloud_xyzrgb);
	h_on_cloud_normals.setup(boost::bind(&CloudViewer::on_cloud_normals, this));
	registerHandler("on_cloud_normals", &h_on_cloud_normals);
	addDependency("on_cloud_normals", &in_cloud_normals);
	h_on_spin.setup(boost::bind(&CloudViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);
}

bool CloudViewer::onInit() {

	viewer = new pcl::visualization::PCLVisualizer (prop_window_name);
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "sample cloud");
    //Eigen::Affine3f tf;
    //string id ="Cloud1";
    viewer->addCoordinateSystem (1.0, prop_window_name, 0);
	viewer->initCameraParameters ();

	return true;
}

bool CloudViewer::onFinish() {
	return true;
}

bool CloudViewer::onStop() {
	return true;
}

bool CloudViewer::onStart() {
	return true;
}

void CloudViewer::on_cloud_xyz() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	viewer->updatePointCloud<pcl::PointXYZ> (cloud, "sample cloud");
}

void CloudViewer::on_cloud_xyzrgb() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution(cloud);
	viewer->removePointCloud("viewcloud") ;
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, color_distribution, "viewcloud") ;
}

void CloudViewer::on_cloud_normals() {
}

void CloudViewer::on_spin() {
	viewer->spinOnce (100);
}



} //: namespace CloudViewer
} //: namespace Processors
