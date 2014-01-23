#ifndef SIFTOBJECTMODELFACTORY_HPP_
#define SIFTOBJECTMODELFACTORY_HPP_

#include <Types/AbstractObjectFactory.hpp> 
#include <Types/SIFTObjectModel.hpp> 


/*!
 * \class SIFTObjectModelFactory
 * \brief Factory responsible for production of SIFT Object Models.
 */
class SIFTObjectModelFactory : public AbstractObjectFactory
{

public:
	SIFTObjectModelFactory(){
		mean_viewpoint_features_number = 0;
	}

	~SIFTObjectModelFactory(){}

	/// Produces and returns a SOM object.
	AbstractObject* produce(){
		SIFTObjectModel *som = new SIFTObjectModel;
		som->cloud = cloud_xyzrgb;
		som->SIFTcloud = cloud_xyzsift;
		som->name = model_name;
		som->mean_viewpoint_features_number = mean_viewpoint_features_number;
		return som;
}
	//add_to_vector(SOM);
	
protected:
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift;// (new pcl::PointCloud<PointXYZSIFT>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb;// (new pcl::PointCloud<pcl::PointXYZ>());

	/// Name of the model.
	std::string model_name;

	/// Mean number of viewpoint features.
	int mean_viewpoint_features_number;
	//vector<SOM>
	
};
#endif /* SIFTOBJECTMODELFACTORY_HPP_ */
