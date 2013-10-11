/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "PlaneGenerator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace PlaneGenerator {

PlaneGenerator::PlaneGenerator(const std::string & name) :
		Base::Component(name)  {

}

PlaneGenerator::~PlaneGenerator() {
}

void PlaneGenerator::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("out_pcl", &out_pcl);
registerStream("out_pcl_ptr", &out_pcl_ptr);
	// Register handlers
	h_Generate.setup(boost::bind(&PlaneGenerator::Generate, this));
	registerHandler("Generate", &h_Generate);
	addDependency("Generate", NULL);

}

bool PlaneGenerator::onInit() {
	

//generate 
  // Fill in the cloud data
  cloud.width  = 15;
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);

  // Generate the data
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1.0;
  }

  // Set a few outliers
  cloud.points[0].z = 2.0;
  cloud.points[3].z = -2.0;
  cloud.points[6].z = 4.0;
	
	out_pcl.write(cloud);	
	cloudPtr = cloud.makeShared();
	out_pcl_ptr.write(cloudPtr);
	

	return true;
}

bool PlaneGenerator::onFinish() {
	return true;
}

bool PlaneGenerator::onStop() {
	return true;
}

bool PlaneGenerator::onStart() {
	return true;
}

void PlaneGenerator::Generate() {

	//out_pcl.write(cloudPtr);
}



} //: namespace PlaneGenerator
} //: namespace Processors
