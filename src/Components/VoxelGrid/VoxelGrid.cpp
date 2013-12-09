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
registerStream("in_pcl", &in_pcl);
registerStream("out_pcl", &out_pcl);
	// Register handlers
	h_filter.setup(boost::bind
(&VoxelGrid::filter, this));
	registerHandler("filter", &h_filter);
	addDependency("filter", &in_pcl);

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
	std::cout << "filter"<<endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_pcl.read();
	std::cout << "filter"<<endl;
	pcl::VoxelGrid<pcl::PointXYZ> vg;///////////////???????????
	std::cout << "filter"<<endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	std::cout << "filter"<<endl;
	vg.setInputCloud (cloud);
	std::cout << "filter"<<endl;
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	std::cout << "filter"<<endl;
	vg.filter (*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;
	out_pcl.write(cloud_filtered);
	
}



} //: namespace VoxelGrid
} //: namespace Processors
