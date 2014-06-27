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

#include <pcl/filters/filter.h>

namespace Processors {
namespace CloudViewer {

CloudViewer::CloudViewer(const std::string & name) :
		Base::Component(name),
    prop_window_name("window_name", std::string("3D PC Viewer")),
    prop_coordinate_system("coordinate_system", true),
    prop_two_viewports("two_viewports", false)
{
  registerProperty(prop_window_name);
  registerProperty(prop_coordinate_system);
  registerProperty(prop_two_viewports);
}

CloudViewer::~CloudViewer() {
}

void CloudViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyz2", &in_cloud_xyz2);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzrgb2", &in_cloud_xyzrgb2);
	registerStream("in_cloud_normals", &in_cloud_normals);

	// Register handlers
	h_on_cloud_xyz.setup(boost::bind(&CloudViewer::on_cloud_xyz, this));
	registerHandler("on_cloud_xyz", &h_on_cloud_xyz);
	addDependency("on_cloud_xyz", &in_cloud_xyz);
	h_on_clouds_xyz.setup(boost::bind(&CloudViewer::on_clouds_xyz, this));
	registerHandler("on_clouds_xyz", &h_on_clouds_xyz);
	addDependency("on_clouds_xyz", &in_cloud_xyz);
	addDependency("on_clouds_xyz", &in_cloud_xyz2);
	h_on_cloud_xyzrgb.setup(boost::bind(&CloudViewer::on_cloud_xyzrgb, this));
	registerHandler("on_cloud_xyzrgb", &h_on_cloud_xyzrgb);
	addDependency("on_cloud_xyzrgb", &in_cloud_xyzrgb);
	h_on_clouds_xyzrgb.setup(boost::bind(&CloudViewer::on_clouds_xyzrgb, this));
	registerHandler("on_clouds_xyzrgb", &h_on_clouds_xyzrgb);
	addDependency("on_clouds_xyzrgb", &in_cloud_xyzrgb);
	addDependency("on_clouds_xyzrgb", &in_cloud_xyzrgb2);
	h_on_cloud_normals.setup(boost::bind(&CloudViewer::on_cloud_normals, this));
	registerHandler("on_cloud_normals", &h_on_cloud_normals);
	addDependency("on_cloud_normals", &in_cloud_normals);
	h_on_spin.setup(boost::bind(&CloudViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);
}

bool CloudViewer::onInit() {

	if(prop_two_viewports){
		cout<< LOG(LTRACE) << "CloudViewer::onInit, prop_two_viewports==true\n";
		viewer = new pcl::visualization::PCLVisualizer (prop_window_name);
		v1 = 0;
		v2 = 1;
		viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
		viewer->setBackgroundColor (0, 0, 0, v1);
		
		viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
		viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);			
	}
	else{
		cout<< LOG(LTRACE) << "CloudViewer::onInit, prop_two_viewports==false\n";
		viewer = new pcl::visualization::PCLVisualizer (prop_window_name);
		viewer->setBackgroundColor (0, 0, 0);
		if(prop_coordinate_system)
		viewer->addCoordinateSystem (1.0, 0);
		viewer->addPointCloud<pcl::PointXYZ> (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), "sample cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "sample cloud");
	}
	if(prop_coordinate_system) {
#if PCL_VERSION_COMPARE(>=,1,7,1)
		viewer->addCoordinateSystem (1.0, "ClustersViewer", 0);
#else
		viewer->addCoordinateSystem (1.0);
#endif
	}
		
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "sample cloud");
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
	LOG(LTRACE) << "CloudViewer::on_cloud_xyz\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	viewer->updatePointCloud<pcl::PointXYZ> (cloud, "sample cloud");
}

void CloudViewer::on_clouds_xyz() {
	if(!prop_two_viewports)
		LOG(LDEBUG) << "Set property two_viewports = 1\n";
	LOG(LTRACE) << "CloudViewer::on_clouds_xyz\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = in_cloud_xyz2.read();

	viewer->removePointCloud("viewcloud",v1) ;
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "viewcloud", v1);
	
	viewer->removePointCloud("viewcloud2",v2) ;
	viewer->addPointCloud<pcl::PointXYZ> (cloud2, "viewcloud2", v2);
}

void CloudViewer::on_cloud_xyzrgb() {
	LOG(LTRACE) << "CloudViewer::on_cloud_xyzrgb\n";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	
	// Filter the NaN points.
	std::vector<int> indices;
	cloud->is_dense = false; 
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution(cloud);
	viewer->removePointCloud("viewcloud") ;
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, color_distribution, "viewcloud") ;
}

void CloudViewer::on_clouds_xyzrgb() {
	if(!prop_two_viewports)
		LOG(LDEBUG) << "Set property two_viewports = 1\n";
	LOG(LTRACE) << "CloudViewer::on_clouds_xyzrgb\n";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = in_cloud_xyzrgb2.read();

	std::vector<int> indices;
	cloud->is_dense = false; 
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	std::vector<int> indices2;
	cloud2->is_dense = false; 
	pcl::removeNaNFromPointCloud(*cloud2, *cloud2, indices2);
	

	viewer->removePointCloud("viewcloud",v1) ;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "viewcloud", v1);
	
	viewer->removePointCloud("viewcloud2",v2) ;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2 (cloud2);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, rgb2, "viewcloud2", v2);
}

void CloudViewer::on_cloud_normals() {
}

void CloudViewer::on_spin() {
	viewer->spinOnce (100);
}



} //: namespace CloudViewer
} //: namespace Processors
