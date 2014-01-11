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
	
 // Neighbors within radius search
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

	int t;//times
	cout<<"cloud size: "<<cloud->size()<<endl;
	//zliczenie krotnosci
	//0 - punkt nie ma jeszcze policzonej krotnosci
	//-1 - punkt do usuniecia
  pcl::PointCloud<PointXYZSIFT>::iterator pt_iter = cloud->begin();
  while(pt_iter!=cloud->end()){
	
	  searchPoint = *pt_iter++;
	  if(searchPoint.times!=0)
		continue;
	  t = kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);


	  if ( t > 0 )
	  {
		int wrongPoints = 0;
		for (size_t i = 0; i < t; ++i){
			if(cloud->points[ pointIdxRadiusSearch[i] ].times ==0)
				cloud->points[ pointIdxRadiusSearch[i] ].times = -1;
			else //punkt zostal juz usuniety/zliczony wczesniej
				wrongPoints++;
				//cloud->erase(cloud->begin() + pointIdxRadiusSearch[i]);//-1??
		  //std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
					//<< " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
					//<< " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
					//<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
		}
		cloud->points[ pointIdxRadiusSearch[0] ].times = t-wrongPoints;
		//cout<< "t: " <<t;
		//if(wrongPoints>0)
			//cout<<" Wrong points: "<<wrongPoints;
		//cout<<endl;
	  }
 }

	//usuniecie nadmiarowych punktow
	pt_iter = cloud->begin();
	 while(pt_iter!=cloud->end()){
		 if(pt_iter->times==-1 || pt_iter->times==0){
			 pt_iter = cloud->erase(pt_iter);
		}
		else{
			++pt_iter;	
		}
		
	}
	cout<<"cloud size: "<<cloud->size()<<endl;
	
	out_cloud_xyzsift.write(cloud);
}



} //: namespace Downsampling
} //: namespace Processors
