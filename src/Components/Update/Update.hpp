/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef UPDATE_HPP_
#define UPDATE_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointXYZSIFT.hpp> 
#include <Types/SIFTObjectModel.hpp> 

#include <opencv2/core/core.hpp>

namespace Processors {
namespace Update {

/*!
 * \class Update
 * \brief Update processor class.
 *
 * Update processor.
 */
class Update: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Update(const std::string & name = "Update");

	/*!
	 * Destructor
	 */
	virtual ~Update();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


// Input data streams

		Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;
		Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;

// Output data streams

		Base::DataStreamOut<SIFTObjectModel> out_instance;
		
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud;
	// Handlers
	Base::EventHandler2 h_update;

	
	// Handlers
	void update();
	
	
	
	int counter;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merged;
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift_merged;
	Eigen::Matrix4f global_trans;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prev ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_next ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_merge;
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift_prev;
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift_next;
	
	

};

} //: namespace Update
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Update", Processors::Update::Update)

#endif /* UPDATE_HPP_ */
