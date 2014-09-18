/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef PCDWRITER_HPP_
#define PCDWRITER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//#include <Types/PointXYZSIFT.hpp>

namespace Processors {
namespace PCDWriter {

/*!
 * \class PCDWrite
 * \brief PCDWrite processor class.
 *
 * PCDWrite processor.
 */
class PCDWriter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	PCDWriter(const std::string & name = "PCDWriter");

	/*!
	 * Destructor
	 */
	virtual ~PCDWriter();

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
        Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr > in_cloud_xyz;
		Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_cloud_xyzrgb;
// Output data streams

	// Handlers
    Base::EventHandler2 h_Write_xyz;
    Base::EventHandler2 h_Write_xyzrgb;
	
	Base::Property<std::string> filename;
    Base::Property<bool> binary;
	
	// Handlers
    void Write_xyz();
    void Write_xyzrgb();

};

} //: namespace PCDWrite
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("PCDWriter", Processors::PCDWriter::PCDWriter)

#endif /* PCDWRITER_HPP_ */
