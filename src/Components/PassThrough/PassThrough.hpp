/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef PASSTHROUGH_HPP_
#define PASSTHROUGH_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

namespace Processors {
namespace PassThrough {

/*!
 * \class PassThrough
 * \brief PassThrough processor class.
 *
 * PassThrough processor.
 */
class PassThrough: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	PassThrough(const std::string & name = "PassThrough");

	/*!
	 * Destructor
	 */
	virtual ~PassThrough();

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

		Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud;

// Output data streams

		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud;
	// Handlers
	Base::EventHandler2 h_filter;
		Base::Property<float> xa;
		Base::Property<float> xb;
		Base::Property<float> ya;
		Base::Property<float> yb;
		Base::Property<float> za;
		Base::Property<float> zb;
		Base::Property<bool> negative;

	
	// Handlers
	void filter();

};

} //: namespace PassThrough
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("PassThrough", Processors::PassThrough::PassThrough)

#endif /* PASSTHROUGH_HPP_ */
