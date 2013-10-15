/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "PCDWriter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace Processors {
namespace PCDWriter {

PCDWriter::PCDWriter(const std::string & name) :
		Base::Component(name),
		filename("filename", std::string(""))   {
			registerProperty(filename);
}

PCDWriter::~PCDWriter() {
}

void PCDWriter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_pcl", &in_pcl);
	// Register handlers
	h_Write.setup(boost::bind(&PCDWriter::Write, this));
	registerHandler("Write", &h_Write);
	//addDependency("Write", &in_pcl);

}

bool PCDWriter::onInit() {

	return true;
}

bool PCDWriter::onFinish() {
	return true;
}

bool PCDWriter::onStop() {
	return true;
}

bool PCDWriter::onStart() {
	return true;
}

void PCDWriter::Write() {
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	
	cloud = in_pcl.read();
	pcl::io::savePCDFileASCII (filename, cloud);
	std::cerr << "Saved " << cloud.points.size () << " data points to test2_pcd.pcd." << std::endl;
	
}



} //: namespace PCDWrite
} //: namespace Processors
