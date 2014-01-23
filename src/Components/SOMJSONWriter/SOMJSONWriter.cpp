/*!
 * \file
 * \brief
 * \author tkornuta,,,
 */

#include <memory>
#include <string>

#include "SOMJSONWriter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace SOMJSONWriter {

SOMJSONWriter::SOMJSONWriter(const std::string & name) :
		Base::Component(name),
		dir("directory", boost::bind(&SOMJSONWriter::onDirChanged, this, _1, _2), "./"),
		SOMname("SOM", boost::bind(&SOMJSONWriter::onSOMNameChanged, this, _1, _2), "SOM")
{
	CLOG(LTRACE) << "Hello SOMJSONWriter\n";
	registerProperty(SOMname);
	registerProperty(dir);
}


SOMJSONWriter::~SOMJSONWriter() {
	CLOG(LTRACE) << "Bye SOMJSONWriter\n";
}

void SOMJSONWriter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_som", &in_som);
	// Register handlers
	h_Write.setup(boost::bind(&SOMJSONWriter::Write, this));
	registerHandler("Write", &h_Write);
}

bool SOMJSONWriter::onInit() {

	return true;
}

bool SOMJSONWriter::onFinish() {
	return true;
}

bool SOMJSONWriter::onStop() {
	return true;
}

bool SOMJSONWriter::onStart() {
	return true;
}

void SOMJSONWriter::onSOMNameChanged(const std::string & old_SOMname, const std::string & new_SOMname) {
	SOMname = new_SOMname;
	CLOG(LTRACE) << "onSOMNameChanged: " << std::string(SOMname) << std::endl;
}

void SOMJSONWriter::onDirChanged(const std::string & old_dir,
		const std::string & new_dir) {
	dir = new_dir;
	CLOG(LTRACE) << "onDirChanged: " << std::string(dir) << std::endl;
}


void SOMJSONWriter::Write() {
	// Read model from data stream.
	if (in_som.empty()) {
		CLOG(LWARNING) << "The SOM datastream is empty.";
	}
	// Get SOM.
	SIFTObjectModel* som = in_som.read();

	// Save point cloud.
	std::string name_cloud = std::string(dir) + std::string("/") + std::string(SOMname) + std::string("_xyz.pcd");
	pcl::io::savePCDFileASCII (name_cloud, *(som->cloud));
	CLOG(LTRACE) << "Write: saved " << som->cloud->points.size () << " cloud points to "<< name_cloud;

	// Save feature cloud.
	std::string name_SIFTcloud = std::string(dir) + std::string("/") + std::string(SOMname) + std::string("_xyzSIFT.pcd");
	pcl::io::savePCDFileASCII (name_SIFTcloud, *(som->SIFTcloud));
	CLOG(LTRACE) << "Write: saved " << som->SIFTcloud->points.size () << " feature points to "<< name_SIFTcloud;

	// Save JSON model description.
	ptree ptree_file;
	ptree_file.put("name", som->name);
	ptree_file.put("type", "SIFTObjectModel");
	ptree_file.put("mean_viewpoint_features_number", som->mean_viewpoint_features_number);
	ptree_file.put("cloud_pcd", name_cloud);
	ptree_file.put("SIFTcloud_pcd", name_SIFTcloud);
	write_json (std::string(dir) + std::string("/") + std::string(SOMname) + std::string(".json"), ptree_file);
} 


} //: namespace SOMJSONWriter
} //: namespace Processors
