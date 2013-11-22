/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef JSONWRITER_HPP_
#define JSONWRITER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Types/PointXYZRGBSIFT.hpp> 

namespace Processors {
namespace JSONWriter {

/*!
 * \class JSONWriter
 * \brief JSONWriter processor class.
 *
 * JSONWriter processor.
 */
class JSONWriter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	JSONWriter(const std::string & name = "JSONWriter");

	/*!
	 * Destructor
	 */
	virtual ~JSONWriter();

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
	
	bool replace(std::string& str, const std::string& from, const std::string& to);


// Input data streams

		Base::DataStreamIn<pcl::PointCloud<PointXYZRGBSIFT>::Ptr> in_cloud_xyzrgbsift;
		Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_cloud_xyz;
		Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;

// Output data streams

	// Handlers
	Base::EventHandler2 h_write_xyz;
	Base::EventHandler2 h_write_xyzrgb;
	Base::EventHandler2 h_write_xyzrgbsift;

	Base::Property<std::string> filename;
	
	// Handlers
	void write_xyz();
	void write_xyzrgb();
	void write_xyzrgbsift();

};

} //: namespace JSONWriter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("JSONWriter", Processors::JSONWriter::JSONWriter)

#endif /* JSONWRITER_HPP_ */
