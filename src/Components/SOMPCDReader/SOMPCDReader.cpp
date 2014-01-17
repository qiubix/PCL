/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "SOMPCDReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/algorithm/string/split.hpp>

namespace Processors {
namespace SOMPCDReader{

/* SOM PCD Reader : SOMF */
SOMPCDReader::SOMPCDReader(const std::string & name) :
		Base::Component(name) , 
		names("names", std::string(" ")) {
		registerProperty(names);//nazwy katalogow oddzielone ;

}

SOMPCDReader::~SOMPCDReader() {
}

void SOMPCDReader::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("out_models", &out_models);
	// Register handlers
	//h_produce.setup(boost::bind(&SOMPCDReader::produce, this));
	//registerHandler("produce", &h_produce);
	//addDependency("produce", NULL);
	h_loadModels.setup(boost::bind(&SOMPCDReader::loadModels, this));
	registerHandler("loadModels", &h_loadModels);
	addDependency("loadModels", NULL);

}

bool SOMPCDReader::onInit() {
	cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
	return true;
}

bool SOMPCDReader::onFinish() {
	return true;
}

bool SOMPCDReader::onStop() {
	return true;
}

bool SOMPCDReader::onStart() {
	return true;
}

/*void SIFTObjectModelFactory::produce() {
}*/

void SOMPCDReader::loadModels() {
	
    //std::vector<AbstractObject> models;
    std::vector<AbstractObject*> models;
	
	std::vector<std::string> namesList;
	string s= names;
	boost::split(namesList, s, boost::is_any_of(";"));
	
	for (size_t i = 0; i < namesList.size(); i++){
		std::vector<std::string> name_split;
		boost::split(name_split, namesList[i], boost::is_any_of("/"));
		model_name = name_split[name_split.size()-1];
		string name_xyz = namesList[i] + "/" + name_split[name_split.size()-1] + "_xyzrgb.pcd";
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (name_xyz, *cloud_xyzrgb) == -1) //* load the file
	    {
            cout <<"Niepoprawny model"<<endl;
			continue;
	    }
		string name_xyzsift = namesList[i] + "/" +  name_split[name_split.size()-1] + "_xyzsift.pcd";
		if (pcl::io::loadPCDFile<PointXYZSIFT> (name_xyzsift, *cloud_xyzsift) == -1) //* load the file
	    {
            cout <<"Niepoprawny model"<<endl;
			continue;
	    }

		//dodanie do wektora modeli	    
		SIFTObjectModel* model = new SIFTObjectModel();
		*model = dynamic_cast<SIFTObjectModel>(produce());
		models.push_back(model);
        //models.push_back(produce());
    }

    out_models.write(models); /////////terminate called after throwing an instance of 'std::bad_alloc' what():  std::bad_alloc Przerwane (core dumped)

}



} //: namespace SOMPCDReader
} //: namespace Processors
