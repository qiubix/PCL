/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef FLANN_HPP_
#define FLANN_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/PointXYZSIFT.hpp> 
#include <Types/SIFTObjectModel.hpp> 

#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace FLANN {

/*!
 * \class FLANN
 * \brief FLANN processor class.
 *
 * FLANN processor.
 */
class FLANN: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	FLANN(const std::string & name = "FLANN");

	/*!
	 * Destructor
	 */
	virtual ~FLANN();

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

		Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_pcl_model;
		Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_pcl;

// Output data streams

	// Handlers
	Base::EventHandler2 h_readModel;
	Base::EventHandler2 h_match;

	
	// Handlers
	void readModel();
	void match();
	
	pcl::PointCloud<PointXYZSIFT>::Ptr model;

};

} //: namespace FLANN
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("FLANN", Processors::FLANN::FLANN)

#endif /* FLANN_HPP_ */
