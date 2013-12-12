#ifndef SIFTOBJECTMODEL_HPP_
#define SIFTOBJECTMODEL_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointCloudObject.hpp> 
#include <Types/PointXYZSIFT.hpp> 
//namespace Types {

class SIFTObjectModel : public PointCloudObject
{
	public:
	pcl::PointCloud<PointXYZSIFT>::Ptr SIFTcloud;
};


//} //: namespace Types

#endif /* SIFTOBJECTMODEL_HPP_ */
