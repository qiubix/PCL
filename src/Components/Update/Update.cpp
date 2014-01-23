/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "Update.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
///////////////////////////////////////////
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include "pcl/impl/instantiate.hpp"
#include "pcl/search/kdtree.h" 
#include "pcl/search/impl/kdtree.hpp"
#include <pcl/registration/correspondence_estimation.h>
#include "pcl/registration/correspondence_rejection_sample_consensus.h"

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
////////////////////////////////////////////////////////////////////////
#include <pcl/filters/filter.h>


namespace Processors {
namespace Update {

class SIFTFeatureRepresentation: public pcl::DefaultFeatureRepresentation <PointXYZSIFT> //could possibly be pcl::PointRepresentation<...> ??
{
	using pcl::PointRepresentation<PointXYZSIFT>::nr_dimensions_;
	public:
	SIFTFeatureRepresentation ()
	{
		// Define the number of dimensions
		nr_dimensions_ = 128 ;
		trivial_ = false ;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray (const PointXYZSIFT &p, float * out) const
	{
		//This representation is only for determining correspondences (not for use in Kd-tree for example - so use only SIFT part of the point	
		for (register int i = 0; i < 128 ; i++)
			out[i] = p.descriptor[i];//p.descriptor.at<float>(0, i) ;
		//std::cout << "SIFTFeatureRepresentation:copyToFloatArray()" << std::endl ;
	}
};



Eigen::Matrix4f computeTransformationSAC(const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_src, const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_trg, 
		const pcl::CorrespondencesConstPtr& correspondences, pcl::Correspondences& inliers)
{
	std::cout << "Computing SAC" << std::endl ;	
	std::cout.flush() ;
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZSIFT> sac ;
	sac.setInputSource(cloud_src) ;
	sac.setInputTarget(cloud_trg) ;
	sac.setInlierThreshold(0.001f) ;
	sac.setMaximumIterations(2000) ;
	sac.setInputCorrespondences(correspondences) ;
	sac.getCorrespondences(inliers) ;
	std::cout << "SAC inliers " << inliers.size() << std::endl ;
	if ( ((float)inliers.size()/(float)correspondences->size()) >85)
		return Eigen::Matrix4f::Identity();
	return sac.getBestTransformation() ;
}


///////////////////////////////////////////////////////////////////
Update::Update(const std::string & name) :
		Base::Component(name)  {

}

Update::~Update() {
}

void Update::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
registerStream("out_instance", &out_instance);
registerStream("out_cloud", &out_cloud);
registerStream("out_cloud_sift", &out_cloud_sift);
	// Register handlers
	h_update.setup(boost::bind(&Update::update, this));
	registerHandler("update", &h_update);
	addDependency("update", &in_cloud_xyzsift);
	addDependency("update", &in_cloud_xyzrgb);

}

bool Update::onInit() {
	counter = 0;
	global_trans = Eigen::Matrix4f::Identity();

	cloud_merged = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud_sift_merged = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
	cloud_prev = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud_next = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud_to_merge = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud_sift_to_merge = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
	cloud_sift_prev = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
	cloud_sift_next = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());	
	
	return true;
}

bool Update::onFinish() {
	return true;
}

bool Update::onStop() {
	return true;
}

bool Update::onStart() {
	return true;
}

void Update::update() {
	cout<<"Update()"<<endl;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift = in_cloud_xyzsift.read();
	
	//cout<< "cloud->size(): "<<cloud->size()<<endl;
	//first clouds
	if (counter == 0 ){
		std::vector<int> indices;
		
		*cloud_merged = *cloud;
		cloud_merged->is_dense = false; 
		pcl::removeNaNFromPointCloud(*cloud_merged,*cloud_merged,indices);
		*cloud_prev = *cloud_merged;
		*cloud_sift_merged = *cloud_sift;
		*cloud_sift_prev = *cloud_sift;
		counter++;
		out_cloud.write(cloud_merged);
		return;
	}
	
	std::vector<int> indices;
	
	*cloud_next = *cloud;
	cloud_next->is_dense = false; 
	pcl::removeNaNFromPointCloud(*cloud_next,*cloud_next,indices);	
	*cloud_sift_next = *cloud_sift;
	counter++;
	//cout<< "cloud_next->size(): "<<cloud_next->size()<<endl;
	
	
	
		pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
		pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst ;
	
		//SIFTFeatureRepresentation point_representation ;
		//correst.setPointRepresentation (point_representation.makeShared()); //NEVER do like this, makeShared will return DefaultFeatureRepresentation<PointDefault>!
		SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation()) ;
		correst.setPointRepresentation(point_representation) ;
		correst.setInputSource(cloud_sift_next) ;
		correst.setInputTarget(cloud_sift_merged) ;//cloud_sift_prev
		correst.determineReciprocalCorrespondences(*correspondences) ;
		std::cout << "\nNumber of reciprocal correspondences: " << correspondences->size() << " out of " << cloud_sift_next->size() << " keypoints" << std::endl ;


//zliczanie krotnosci 	

		for(int i = 0; i< correspondences->size();i++){	
			if (correspondences->at(i).index_query >=cloud_sift_next->size() ||
				correspondences->at(i).index_match >=cloud_sift_merged->size()){
					//
					continue;
			}
				
			//cout<< correspondences->at(i).index_query << " ";
			//cout<< correspondences->at(i).index_match << " ";
			//cout<< correspondences->at(i).distance << " ";
			//cout<<cloud_sift_prev->at(correspondences->at(i).index_query).times <<endl;
			//for(int j=0;j<128;j++){
				//cout<<cloud_sift_prev->at(correspondences->at(i).index_query).descriptor[j]<<" ";
				//cout<<cloud_sift_next->at(correspondences->at(i).index_match).descriptor[j]<<", ";
				////cout<<correspondences->at(i).distance<<" ";
			//}
			//cout<<endl;
			//if(cloud_sift_merged->at(correspondences->at(i).index_query).times==0)
				//cloud_sift_merged->at(correspondences->at(i).index_query).times++;
			//cloud_sift_merged->at(correspondences->at(i).index_query).times++;
			cloud_sift_next->at(correspondences->at(i).index_query).times = cloud_sift_merged->at(correspondences->at(i).index_match).times + 1;
			cloud_sift_merged->at(correspondences->at(i).index_match).times=-1; //poprzedni punkt do usuniecia			
		}
///////////	
	
		//for (pcl::PointCloud<PointXYZRGBSIFT>::iterator pt_iter = cloud_sift_next->begin(); pt_iter != cloud_sift_next->end() ; pt_iter++) {
		//	displayMatrixInfo(pt_iter->descriptor) ;			
		//	std::cout << "Matrix value " << pt_iter->descriptor << std::endl ;
		//}
		
		//displayCorrespondences(cloud_next, cloud_sift_next, cloud_prev, cloud_sift_prev, correspondences, viewer) ;

		//Compute transformation between clouds and update global transformation of cloud_next	
		pcl::Correspondences inliers ;
		Eigen::Matrix4f current_trans = computeTransformationSAC(cloud_sift_next, cloud_sift_merged, correspondences, inliers) ; //_prev
		if (current_trans == Eigen::Matrix4f::Identity()){
			out_cloud.write(cloud_merged);
			out_cloud_sift.write(cloud_sift_merged);
			return;
		}
		std::cout << "Transformation cloud_next -> cloud_prev: " << std::endl << current_trans << std::endl ;
		global_trans = global_trans * current_trans ;
		std::cout << "Global transformation : " << std::endl << global_trans << std::endl ;


		//Delete points
		pcl::PointCloud<PointXYZSIFT>::iterator pt_iter = cloud_sift_merged->begin();
		while(pt_iter!=cloud_sift_merged->end()){
			if(pt_iter->times==-1){
				pt_iter = cloud_sift_merged->erase(pt_iter);
			}
			else{
				++pt_iter;	
			}
		}

		//Merge cloud - cloud_next
		pcl::transformPointCloud(*cloud_next, *cloud_to_merge, current_trans) ; //global_trans
		pcl::transformPointCloud(*cloud_sift_next, *cloud_sift_to_merge, current_trans) ; //global_trans
		//addCloudToScene(cloud_to_merge, sceneviewer, counter - 1) ; 
		*cloud_merged = *cloud_merged + *cloud_to_merge ;
		*cloud_sift_merged = *cloud_sift_merged + *cloud_sift_to_merge ;

		*cloud_prev = *cloud_next ;
		*cloud_sift_prev = *cloud_sift_next ;
			
		cloud_xyzrgb = cloud_merged;
		cloud_xyzsift = cloud_sift_merged;
		
		out_instance.write(produce());	
	
		out_cloud.write(cloud_merged);
		out_cloud_sift.write(cloud_sift_merged);
}



} //: namespace Update
} //: namespace Processors
