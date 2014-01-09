#ifndef POINTCLOUDOBJECT_HPP_
#define POINTCLOUDOBJECT_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/AbstractObject.hpp> 

//namespace Types {

class PointCloudObject : public AbstractObject
{
	public:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
};


//} //: namespace Types

#endif /* POINTCLOUDOBJECT_HPP_ */
