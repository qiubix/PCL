/*!
 * \file
 * \brief Contains body of  the class (component) able to display may XYZ point clouds in one window.
 * \author Tomasz Kornuta [tkornuta@gmail.com]
 */

#ifndef XYZCLOUDVIEWER_HPP_
#define XYZCLOUDVIEWER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include <pcl/visualization/pcl_visualizer.h>


namespace Processors {
namespace XYZCloudViewer {

/*!
 * \class XYZCloudViewer
 * \brief XYZCloudViewer processor class - component able to display may XYZ point clouds in one window.
 *
 * XYZCloudViewer processor.
 */
class XYZCloudViewer: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	XYZCloudViewer(const std::string & name = "XYZCloudViewer");

	/*!
	 * Destructor
	 */
	virtual ~XYZCloudViewer();

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

	// Data streams
	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZ>::Ptr > in_cloud_xyz;

	// Data streams
	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZ>::Ptr > in_cloud_xyz1;

	// Handlers
	Base::EventHandler2 h_on_cloud_xyz;
	Base::EventHandler2 h_on_spin;

	
	// Handlers
	void on_cloud_xyz();
	void on_spin();

	// Property enabling to change the name of displayed window.
	Base::Property<std::string> prop_title;

	/// Point cloud viewer.
	pcl::visualization::PCLVisualizer * viewer;
	
};

} //: namespace XYZCloudViewer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("XYZCloudViewer", Processors::XYZCloudViewer::XYZCloudViewer)

#endif /* XYZCLOUDVIEWER_HPP_ */
