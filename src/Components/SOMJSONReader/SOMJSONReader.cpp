/*!
 * \file
 * \brief
 * \author tkornuta,,,
 */

#include <memory>
#include <string>

#include "SOMJSONReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace SOMJSONReader {

SOMJSONReader::SOMJSONReader(const std::string & name) :
		Base::Component(name) , 
		filenames("filenames", boost::bind(&SOMJSONReader::onFilenamesChanged, this, _1, _2), "")
{
	registerProperty(filenames);

}

SOMJSONReader::~SOMJSONReader() {
}

void SOMJSONReader::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_models", &out_models);

	// Register handlers
	h_loadModels.setup(boost::bind(&SOMJSONReader::loadModels, this));
	registerHandler("loadModels", &h_loadModels);

}

bool SOMJSONReader::onInit() {
	LOG(LTRACE) << "SOMJSONReader::onInit()";
	// Load models at start.
	loadModels();
	return true;
}

bool SOMJSONReader::onFinish() {
	return true;
}

bool SOMJSONReader::onStop() {
	return true;
}

bool SOMJSONReader::onStart() {
	return true;
}

void SOMJSONReader::loadModels() {
	LOG(LTRACE) << "SOMJSONReader::loadModels()";

	// List of the returned SOMs.
	std::vector<AbstractObject*> models;
	
	// Names of models/JSON files.	
	std::vector<std::string> namesList;
	string s= filenames;
	boost::split(namesList, s, boost::is_any_of(";"));
	
	// Iterate through JSON files.
	for (size_t i = 0; i < namesList.size(); i++){
		ptree ptree_file;
		try{
			read_json(namesList[i], ptree_file);
			}
		catch(std::exception const& e){
			LOG(LERROR) << "SOMJSONReader: file "<< namesList[i] <<" not found\n";
			return;	
		}		

	}
/*	for (size_t i = 0; i < namesList.size(); i++){
		std::vector<std::string> name_split;
		boost::split(name_split, namesList[i], boost::is_any_of("/"));
		model_name = name_split[name_split.size()-1];
		string name_xyz = namesList[i] + "/" + name_split[name_split.size()-1] + "_xyzrgb.pcd";
		cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (name_xyz, *cloud_xyzrgb) == -1) //* load the file
	    {
            cout <<"Niepoprawny model"<<endl;
			continue;
	    }
		string name_xyzsift = namesList[i] + "/" +  name_split[name_split.size()-1] + "_xyzsift.pcd";
		cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
		if (pcl::io::loadPCDFile<PointXYZSIFT> (name_xyzsift, *cloud_xyzsift) == -1) //* load the file
	    {
            cout <<"Niepoprawny model"<<endl;
			continue;
	    }

		//dodanie do wektora modeli	    
		SIFTObjectModel* model;// = new SIFTObjectModel();
		model = dynamic_cast<SIFTObjectModel*>(produce());
		models.push_back(model);
        //models.push_back(produce());
    }

    out_models.write(models); /////////terminate called after throwing an instance of 'std::bad_alloc' what():  std::bad_alloc Przerwane (core dumped)
*/

}


void SOMJSONReader::onFilenamesChanged(const std::string & old_filenames, const std::string & new_filenames) {
	filenames = new_filenames;
	CLOG(LTRACE) << "onFilenamesChanged: " << std::string(filenames) << std::endl;
}

} //: namespace SOMJSONReader
} //: namespace Processors
