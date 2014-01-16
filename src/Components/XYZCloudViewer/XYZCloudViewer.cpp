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
    prop_window_name("window_name", std::string("3D PC Viewer"))
{
  registerProperty(prop_window_name);
}


XYZCloudViewer::~XYZCloudViewer() {
}

void XYZCloudViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);

	// Register handlers
	h_on_cloud_xyz.setup(boost::bind(&XYZCloudViewer::on_cloud_xyz, this));
	registerHandler("on_cloud_xyz", &h_on_cloud_xyz);
	addDependency("on_cloud_xyz", &in_cloud_xyz);

	h_on_spin.setup(boost::bind(&XYZCloudViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);
}

bool XYZCloudViewer::onInit() {

	viewer = new pcl::visualization::PCLVisualizer (prop_window_name);
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

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
}


void XYZCloudViewer::on_spin() {
	viewer->spinOnce (100);
}



} //: namespace XYZCloudViewer
} //: namespace Processors
