/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "PCDReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>


namespace Processors {
namespace PCDReader {

PCDReader::PCDReader(const std::string & name) :
		Base::Component(name), 
		filename("filename", std::string("")) 
		{
		registerProperty(filename);

}

PCDReader::~PCDReader() {
}

void PCDReader::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_pcl", &out_pcl);
	// Register handlers
	h_Read.setup(boost::bind(&PCDReader::Read, this));
	registerHandler("Read", &h_Read);
	//addDependency("Read", NULL);

}

bool PCDReader::onInit() {

	return true;
}

bool PCDReader::onFinish() {
	return true;
}

bool PCDReader::onStop() {
	return true;
}

bool PCDReader::onStart() {
	return true;
}

void PCDReader::Read() {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//"/home/mlaszkow/pcd/table_scene_lms400.pcd"
	  if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) //* load the file
	  {
		cout <<"Błąd"<<endl;
	  }
	out_pcl.write(cloud);
	
}



} //: namespace PCDReader
} //: namespace Processors
