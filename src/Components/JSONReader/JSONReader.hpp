/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef JSONREADER_HPP_
#define JSONREADER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Types/PointXYZRGBSIFT.hpp> 
#include <opencv2/core/core.hpp>

namespace Processors {
namespace JSONReader {

/*!
 * \class JSONReader
 * \brief JSONReader processor class.
 *
 * JSONReader processor.
 */
class JSONReader: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	JSONReader(const std::string & name = "JSONReader");

	/*!
	 * Destructor
	 */
	virtual ~JSONReader();

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


// Output data streams

		Base::DataStreamOut<pcl::PointCloud<PointXYZRGBSIFT>::Ptr> out_cloud_xyzrgbsift;
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_cloud_xyz;
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	// Handlers
	Base::EventHandler2 h_read;
		Base::Property<std::string> filename;

	
	// Handlers
	void read();

};

} //: namespace JSONReader
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("JSONReader", Processors::JSONReader::JSONReader)

#endif /* JSONREADER_HPP_ */
