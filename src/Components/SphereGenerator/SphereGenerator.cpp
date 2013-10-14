/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "SphereGenerator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace SphereGenerator {

SphereGenerator::SphereGenerator(const std::string & name) :
		Base::Component(name),
		r("r", 1),
		x("x", 0),
		y("y", 0),
		z("z", 0),
		nr_of_points("nr_of_points", 150),
		nr_of_outliers("nr_of_outliers", 10)   {
			registerProperty(r);
			registerProperty(x);
			registerProperty(y);
			registerProperty(z);
			registerProperty(nr_of_points);
			registerProperty(nr_of_outliers);
}

SphereGenerator::~SphereGenerator() {
}

void SphereGenerator::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("out_pcl_ptr", &out_pcl_ptr);
registerStream("out_pcl", &out_pcl);
	// Register handlers

}

bool SphereGenerator::onInit() {

//generate 
 
  // Fill in the cloud data
  cloud.width  = nr_of_points;
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);

  // Generate the data
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x =  rand () / (RAND_MAX + 1.0f) * r;
    if (rand()%2)
		cloud.points[i].x = - cloud.points[i].x;
    cloud.points[i].y =  rand () / (RAND_MAX + 1.0f) * sqrt(r*r- cloud.points[i].x*cloud.points[i].x);
    if (rand()%2)
		cloud.points[i].y = - cloud.points[i].y;
    cloud.points[i].z =  sqrt(r*r - cloud.points[i].x*cloud.points[i].x - cloud.points[i].y*cloud.points[i].y);
    if (rand()%2)
		cloud.points[i].z = - cloud.points[i].z;
		
		cloud.points[i].x+=x;
		cloud.points[i].y+=y;
		cloud.points[i].z+=z;
  }




  // Set a few outliers
   for (int i = 0; i < nr_of_outliers; ++i){
	   cloud.points[i].x += (rand () / (RAND_MAX + 1.0f) * 10) -5;
	   cloud.points[i].y += (rand () / (RAND_MAX + 1.0f) * 10) -5;
	   cloud.points[i].z += (rand () / (RAND_MAX + 1.0f) * 10) -5;
   }
  

	
	out_pcl.write(cloud);	
	cloudPtr = cloud.makeShared();
	out_pcl_ptr.write(cloudPtr);

for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " 
                        << cloud.points[i].y << " " 
                        << cloud.points[i].z << std::endl;	

	return true;
}

bool SphereGenerator::onFinish() {
	return true;
}

bool SphereGenerator::onStop() {
	return true;
}

bool SphereGenerator::onStart() {
	return true;
}



} //: namespace SphereGenerator
} //: namespace Processors
