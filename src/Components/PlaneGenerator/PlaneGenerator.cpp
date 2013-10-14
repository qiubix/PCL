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
		Base::Component(name),
		nr_of_points("nr_of_points", 150),
		nr_of_outliers("nr_of_outliers", 10),
		a("a", 1.0),
		b("b", 0.0),
		c("c", 0.0),
		d("d", 0.0)    {
			
			registerProperty(nr_of_points);
			registerProperty(nr_of_outliers);
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

/*	
  // Fill in the cloud data
  cloud.width  = nr_of_points;
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);

  // Generate the data
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1.0;
  }
 
 */
 
 ///////////////////// 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  cloud->width  = nr_of_points;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

// Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = a;
  coefficients->values[1] = b;
  coefficients->values[2] = c;
  coefficients->values[3] = d;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  // Set a few outliers
   for (int i = 0; i < nr_of_outliers; ++i){
	   cloud_projected->points[i].x += (rand () / (RAND_MAX + 1.0f) * 5) +1;
	   if (rand()%2)
			cloud_projected->points[i].x=-cloud_projected->points[i].x;
	   cloud_projected->points[i].y += (rand () / (RAND_MAX + 1.0f) * 5) +1;
	   if (rand()%2)
			cloud_projected->points[i].y=-cloud_projected->points[i].y;
	   cloud_projected->points[i].z += (rand () / (RAND_MAX + 1.0f) * 5) +1;
	   if (rand()%2)
			cloud_projected->points[i].z=-cloud_projected->points[i].z;
   }
	

	out_pcl_ptr.write(cloud_projected); 
	

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
