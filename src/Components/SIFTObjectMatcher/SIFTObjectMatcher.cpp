/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>
#include <iomanip>

#include "SIFTObjectMatcher.hpp"
#include "Common/Logger.hpp"
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/search/kdtree.h> 
#include <pcl/search/impl/kdtree.hpp>
#include <boost/bind.hpp>

namespace Processors {
namespace SIFTObjectMatcher {

class SIFTFeatureRepresentation: public pcl::DefaultFeatureRepresentation<PointXYZSIFT> //could possibly be pcl::PointRepresentation<...> ??
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
	cout<<"readModels()"<<endl;
	for( int i = 0 ; i<models.size(); i++){
		delete models[i];
	}
	models.clear();
	std::vector<AbstractObject*> abstractObjects = in_models.read();
	for( int i = 0 ; i<abstractObjects.size(); i++){
		cout<<"Name: "<<abstractObjects[i]->name<<endl;
		SIFTObjectModel *model = dynamic_cast<SIFTObjectModel*>(abstractObjects[i]);
		if(model!=NULL)
			models.push_back(model);
		else
			cout<<"niepoprawny model"<<endl;
	}
	cout<<models.size()<<" modeli"<<endl;
}

void SIFTObjectMatcher::match() {
	cout<<"SIFTObjectMatcher::match()"<<endl;
	if(models.empty()){
		cout<<"No models available" <<endl;
		return;
	}
			
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift = in_cloud_xyzsift.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_cloud_xyzrgb.read();	
	

		for (int i = 0 ; i<models.size(); i++){
			cout<<"liczba cech modelu "<<i<<" "<<models[i]->name<<": " <<
				models[i]->SIFTcloud->size()<<endl; 	
		}
		cout<<"liczba cech instancji : " <<
			cloud_xyzsift->size()<<endl; 


        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
        pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst ;

        //SIFTFeatureRepresentation point_representation ;
        //correst.setPointRepresentation (point_representation.makeShared()); //NEVER do like this, makeShared will return DefaultFeatureRepresentation<PointDefault>!
        SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation()) ;
        correst.setPointRepresentation(point_representation) ;
        for (int i = 0 ; i<models.size(); i++){
            correst.setInputSource(cloud_xyzsift) ;
            correst.setInputTarget(models[i]->SIFTcloud) ;
            correst.determineReciprocalCorrespondences(*correspondences) ;
            std::cout << setprecision(2) << fixed;
            float procent = (float)correspondences->size()/(float)cloud_xyzsift->size();
            std::cout << "\nNumber of reciprocal correspondences: " << correspondences->size() << " out of " << cloud_xyzsift->size() << " keypoints of instance, = " ;
            std::cout << procent << ". "<< models[i]->SIFTcloud->size() << " keypoints of model "<< models[i]->name << std::endl ;
        } 
}



} //: namespace SIFTObjectMatcher
} //: namespace Processors
