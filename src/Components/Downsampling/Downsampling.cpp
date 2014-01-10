/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "Downsampling.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace Downsampling {

Downsampling::Downsampling(const std::string & name) :
		Base::Component(name),
		radius("radius", 0.005)  {
			registerProperty(radius);
}

Downsampling::~Downsampling() {
}

void Downsampling::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	// Register handlers
	h_downsample_xyzsift.setup(boost::bind(&Downsampling::downsample_xyzsift, this));
	registerHandler("downsample_xyzsift", &h_downsample_xyzsift);
	addDependency("downsample_xyzsift", &in_cloud_xyzsift);

}

bool Downsampling::onInit() {

	return true;
}

bool Downsampling::onFinish() {
	return true;
}

bool Downsampling::onStop() {
	return true;
}

bool Downsampling::onStart() {
	return true;
}

void Downsampling::downsample_xyzsift() {
	cout<<"RADIUS: "<<radius<<endl;
	
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
	pcl::KdTreeFLANN<PointXYZSIFT> kdtree;
	kdtree.setInputCloud (cloud);
	PointXYZSIFT searchPoint;
	
	
	cout<<"Cloud size: "<<cloud->size()<<endl;
 // Neighbors within radius search
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  int t;//times
  int s = 0;
  pcl::PointCloud<PointXYZSIFT>::iterator pt_iter = cloud->begin();
 // while(pt_iter!=cloud->end()){

	  searchPoint = *pt_iter++;
	  t = kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	  cout<< "Times: "<<t<<endl;
	  s+=t;
	  cout<<"s: "<<s<<endl;
	  if ( t > 0 )
	  {
		for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
			
			if (i==0)
				cloud->points[ pointIdxRadiusSearch[i] ].times = t;
			else	
				cloud->erase(cloud->begin() + pointIdxRadiusSearch[i]);//-1??
		  //std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
					//<< " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
					//<< " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
					//<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;

		}
	  }
 //}
	out_cloud_xyzsift.write(cloud);
}



} //: namespace Downsampling
} //: namespace Processors
