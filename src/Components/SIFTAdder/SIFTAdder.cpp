/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "SIFTAdder.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

//#include <fstream>
#include <pcl/point_representation.h>

#include "pcl/impl/instantiate.hpp"
#include "pcl/search/kdtree.h" 
#include "pcl/search/impl/kdtree.hpp"
#include <pcl/registration/correspondence_estimation.h>
#include "pcl/registration/correspondence_rejection_sample_consensus.h"

namespace Processors {
namespace SIFTAdder {

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

SIFTAdder::SIFTAdder(const std::string & name) :
		Base::Component(name)  {

}

SIFTAdder::~SIFTAdder() {
}

void SIFTAdder::prepareInterface() {
	// Register data streams, events and event handlers HERE!
//registerStream("in_descriptors", &in_descriptors);
//registerStream("out_descriptors", &out_descriptors);
//registerStream("in_cloud", &in_cloud);
    registerStream("in_models", &in_models);
registerStream("out_cloud", &out_cloud);
registerStream("out_modelMultiplicity", &out_modelMultiplicity);
	// Register handlers
	h_add.setup(boost::bind(&SIFTAdder::add, this));
	registerHandler("add", &h_add);
//	addDependency("add", &in_cloud);
    addDependency("add", &in_models);

}

bool SIFTAdder::onInit() {
	cloud = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
	return true;
}

bool SIFTAdder::onFinish() {
	return true;
}

bool SIFTAdder::onStop() {
	//std::fstream plik;
	//plik.open( "/home/mlaszkow/test.txt", std::ios::out );
	//plik<<"Deskryptory:"<<endl;
	//for (int i = 0; i< descriptors.size(); i++)
		//plik<<descriptors[i]<<endl;
	//plik.close();
	return true;
}

bool SIFTAdder::onStart() {
	return true;
}

void SIFTAdder::add() {
    LOG(LDEBUG) << "SIFTAdder::add\n";
    
    models = in_models.read();
    for (unsigned n=0; n<models.size(); ++n) {
    
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_next = dynamic_cast<SIFTObjectModel*>(models.at(n))->SIFTcloud;

	if (cloud->empty()){
		cloud = cloud_next;
		out_cloud.write(cloud);
		return;
	}
	
    std::map<int,int> modelMultiplicity;
		pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
		pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst ;
	
		//SIFTFeatureRepresentation point_representation ;
		//correst.setPointRepresentation (point_representation.makeShared()); //NEVER do like this, makeShared will return DefaultFeatureRepresentation<PointDefault>!
		SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation()) ;
		correst.setPointRepresentation(point_representation) ;
		correst.setInputSource(cloud_next) ;
		correst.setInputTarget(cloud) ;
		correst.determineReciprocalCorrespondences(*correspondences) ;
		std::cout << "\nNumber of reciprocal correspondences: " << correspondences->size() << " out of " << cloud_next->size() << " keypoints" << std::endl ;
	
		//ransac znalezienie blednych dopasowan
		pcl::Correspondences inliers ;
		pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZSIFT> sac ;
		sac.setInputSource(cloud_next) ;
		sac.setInputTarget(cloud) ;
		sac.setInlierThreshold(0.001f) ;
		sac.setMaximumIterations(2000) ;
		sac.setInputCorrespondences(correspondences) ;
		sac.getCorrespondences(inliers) ;
		//usuniecie blednych dopasowan
		pcl::Correspondences::iterator iter_inliers = inliers.begin();
		while(iter_inliers!=inliers.end()){
			pcl::Correspondences::iterator iter_correspondences = correspondences->begin();
			while(iter_correspondences!=correspondences->end()){
				if(iter_correspondences->index_query == iter_inliers->index_query){
					iter_correspondences= correspondences->erase(iter_correspondences);
					break;
				}
				else 
					++iter_correspondences;
			}
			++iter_inliers;	
		}
		
		//zliczanie krotnosci
		for(int i = 0; i< correspondences->size();i++){	
			if (correspondences->at(i).index_query >=cloud_next->size() ||
				correspondences->at(i).index_match >=cloud->size()){
					continue;
			}
            cloud->at(correspondences->at(i).index_match).times += cloud_next->at(correspondences->at(i).index_query).times;
            modelMultiplicity.insert(std::make_pair<int,int>(correspondences->at(i).index_match, cloud_next->at(correspondences->at(i).index_query).times));
            cloud_next->at(correspondences->at(i).index_query).times=-1; //do usuniecia punkt w nowej chmurze, ktory juz jest zarejestrowany w polaczonej chmurze
		}
		
		//usuniecie punktow
		pcl::PointCloud<PointXYZSIFT>::iterator pt_iter = cloud->begin();
		while(pt_iter!=cloud->end()){
			if(pt_iter->times==-1){
				pt_iter = cloud->erase(pt_iter);
			}
			else{
				++pt_iter;	
			}
		} 
        
        for (unsigned k=0; k<cloud_next->size(); ++k) {
            std::pair<int,int> nextMultiplicity = std::make_pair<int,int>(cloud->size()+k, cloud_next->at(k).times);
            modelMultiplicity.insert(nextMultiplicity);
        }
		
		*cloud = *cloud + *cloud_next;
		
        out_modelMultiplicity.write(modelMultiplicity);
		out_cloud.write(cloud);
        modelMultiplicity.clear();
    }
}

} //: namespace SIFTAdder
} //: namespace Processors
