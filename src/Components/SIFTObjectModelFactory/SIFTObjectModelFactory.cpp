/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "SIFTObjectModelFactory.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/algorithm/string/split.hpp>

namespace Processors {
namespace SIFTObjectModelFactory {

SIFTObjectModelFactory::SIFTObjectModelFactory(const std::string & name) :
		Base::Component(name) , 
		names("names", std::string(" ")) {
		registerProperty(names);//nazwy katalogow oddzielone ;

}

SIFTObjectModelFactory::~SIFTObjectModelFactory() {
}

void SIFTObjectModelFactory::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("out_model", &out_model);
	// Register handlers
	h_produce.setup(boost::bind(&SIFTObjectModelFactory::produce, this));
	registerHandler("produce", &h_produce);
	addDependency("produce", NULL);
	h_loadModels.setup(boost::bind(&SIFTObjectModelFactory::loadModels, this));
	registerHandler("loadModels", &h_loadModels);
	addDependency("loadModels", NULL);

}

bool SIFTObjectModelFactory::onInit() {

	return true;
}

bool SIFTObjectModelFactory::onFinish() {
	return true;
}

bool SIFTObjectModelFactory::onStop() {
	return true;
}

bool SIFTObjectModelFactory::onStart() {
	return true;
}

void SIFTObjectModelFactory::produce() {
}

void SIFTObjectModelFactory::loadModels() {
	
	std::vector<std::string> namesList;
	string s= names;
	boost::split(namesList, s, boost::is_any_of(";"));
	
	for (size_t i = 0; i < namesList.size(); i++){
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift (new pcl::PointCloud<PointXYZSIFT>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
		std::vector<std::string> name;
		boost::split(name, namesList[i], boost::is_any_of("/"));
		string name_xyz = namesList[i] + "/" + name[name.size()-1] + "_xyz.pcd";
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (name_xyz, *cloud_xyz) == -1) //* load the file
	    {
			cout <<"Błąd"<<endl;
	    }
		string name_xyzsift = namesList[i] + "/" +  name[name.size()-1] + "_xyzsift.pcd";
		if (pcl::io::loadPCDFile<PointXYZSIFT> (name_xyzsift, *cloud_xyzsift) == -1) //* load the file
	    {
			cout <<"Błąd"<<endl;
	    }
	    
	    //Types::SIFTObjectModel model;
	    //model.cloud = *cloud_xyz;
		
	}
}



} //: namespace SIFTObjectModelFactory
} //: namespace Processors
