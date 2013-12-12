/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "FLANN.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace FLANN {

FLANN::FLANN(const std::string & name) :
		Base::Component(name)  {

}

FLANN::~FLANN() {
}

void FLANN::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_pcl_model", &in_pcl_model);
registerStream("in_pcl", &in_pcl);
	// Register handlers
	h_readModel.setup(boost::bind(&FLANN::readModel, this));
	registerHandler("readModel", &h_readModel);
	addDependency("readModel", &in_pcl_model);
	addDependency("readModel", &in_pcl);
	h_match.setup(boost::bind(&FLANN::match, this));
	registerHandler("match", &h_match);
	addDependency("match", &in_pcl);

}

bool FLANN::onInit() {

	return true;
}

bool FLANN::onFinish() {
	return true;
}

bool FLANN::onStop() {
	return true;
}

bool FLANN::onStart() {
	return true;
}

void FLANN::readModel() {
	cout<<"readModel"<<endl;
	model = in_pcl_model.read();
}

void FLANN::match() {
	cout<<"match"<<endl;
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_pcl.read();
	SIFTObjectModel SOM;
}



} //: namespace FLANN
} //: namespace Processors
