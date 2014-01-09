/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "SIFTObjectMatcher.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace SIFTObjectMatcher {

SIFTObjectMatcher::SIFTObjectMatcher(const std::string & name) :
		Base::Component(name)  {

}

SIFTObjectMatcher::~SIFTObjectMatcher() {
}

void SIFTObjectMatcher::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_models", &in_models);
registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	// Register handlers
	h_readModels.setup(boost::bind(&SIFTObjectMatcher::readModels, this));
	registerHandler("readModels", &h_readModels);
	addDependency("readModels", &in_models);
	h_match.setup(boost::bind(&SIFTObjectMatcher::match, this));
	registerHandler("match", &h_match);
	addDependency("match", &in_cloud_xyzsift);
	addDependency("match", &in_cloud_xyzrgb);

}

bool SIFTObjectMatcher::onInit() {

	return true;
}

bool SIFTObjectMatcher::onFinish() {
	return true;
}

bool SIFTObjectMatcher::onStop() {
	return true;
}

bool SIFTObjectMatcher::onStart() {
	return true;
}

void SIFTObjectMatcher::readModels() {
	//models = in_models.read();
}

void SIFTObjectMatcher::match() {
	//if(models.empty()){
		//cout<<"No models available" <<endl;
		//return;
	//}
			
	//pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift = in_cloud_xyzsift.read();
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_cloud_xyzrgb.read();	
	
	
	//cloud_xyzsift->size();//liczba cech

		//for (int i = 0 ; i<models.size(); i++){
			//cout<<"liczba cech: " <<
			//models[i].SIFTcloud->size()<<endl; //liczba cech
			
		
		//}
}



} //: namespace SIFTObjectMatcher
} //: namespace Processors
