/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "VoxelGrid.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/filters/voxel_grid.h>

namespace Processors {
namespace VoxelGrid {

VoxelGrid::VoxelGrid(const std::string & name) :
		Base::Component(name) , 
		x("LeafSize.x", 0.01f), 
		y("LeafSize.y", 0.01f), 
		z("LeafSize.z", 0.01f) {
		registerProperty(x);
		registerProperty(y);
		registerProperty(z);

}

VoxelGrid::~VoxelGrid() {
}

void VoxelGrid::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);

	// Register handlers
	h_filter.setup(boost::bind(&VoxelGrid::filter, this));
	registerHandler("filter", &h_filter);
	addDependency("filter", &in_cloud_xyzrgb);
}

bool VoxelGrid::onInit() {

	return true;
}

bool VoxelGrid::onFinish() {
	return true;
}

bool VoxelGrid::onStop() {
	return true;
}

bool VoxelGrid::onStart() {
	return true;
}

void VoxelGrid::filter() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	vg.setInputCloud (cloud);
	vg.setLeafSize (x, y, z);
	vg.filter (*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;
	out_cloud_xyzrgb.write(cloud_filtered);
	
}


} //: namespace VoxelGrid
} //: namespace Processors
