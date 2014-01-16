/*!
 * \file
 * \brief Contains methods of the component able to display may XYZ point clouds in one window.
 * \author Tomasz Kornuta [tkornuta@gmail.com]
 */

#include <memory>
#include <string>

#include "XYZCloudViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace XYZCloudViewer {

XYZCloudViewer::XYZCloudViewer(const std::string & name) :
		Base::Component(name),
		prop_title("title", std::string("XYZ Cloud Viewer"))
{
  registerProperty(prop_title);
}


XYZCloudViewer::~XYZCloudViewer() {
}

void XYZCloudViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyz1", &in_cloud_xyz1);

	// Register handlers
	h_on_cloud_xyz.setup(boost::bind(&XYZCloudViewer::on_cloud_xyz, this));
	registerHandler("on_cloud_xyz", &h_on_cloud_xyz);
	addDependency("on_cloud_xyz", &in_cloud_xyz);
	addDependency("on_cloud_xyz", &in_cloud_xyz1);

	h_on_spin.setup(boost::bind(&XYZCloudViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);
}

bool XYZCloudViewer::onInit() {
	// Create visualizer.
	viewer = new pcl::visualization::PCLVisualizer (prop_title);
	viewer->initCameraParameters ();
//	viewer->addCoordinateSystem (1.0);
//	viewer->resetCameraViewpoint ("sample cloud"); 

	// Add clouds.
	viewer->addPointCloud<pcl::PointXYZ> (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 255,0,0, "sample cloud"); 

//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
//  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");

	viewer->addPointCloud<pcl::PointXYZ> (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), "sample cloud1");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 255,255,255, "sample cloud1"); 

/*
pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;

int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
//  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
//  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud1", v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->addCoordinateSystem (1.0);
*/
	return true;
}

bool XYZCloudViewer::onFinish() {
	return true;
}

bool XYZCloudViewer::onStop() {
	return true;
}

bool XYZCloudViewer::onStart() {
	return true;
}

void XYZCloudViewer::on_cloud_xyz() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	viewer->updatePointCloud<pcl::PointXYZ> (cloud, "sample cloud");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 = in_cloud_xyz1.read();
	viewer->updatePointCloud<pcl::PointXYZ> (cloud1, "sample cloud1");
}


void XYZCloudViewer::on_spin() {
	viewer->spinOnce (100);
}



} //: namespace XYZCloudViewer
} //: namespace Processors
