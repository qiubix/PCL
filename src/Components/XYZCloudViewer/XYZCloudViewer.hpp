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

#include <Types/MatrixTranslator.hpp>

namespace Processors {
namespace XYZCloudViewer {

/*!
 * \class XYZCloudViewer
 * \brief XYZCloudViewer processor class - component able to display many XYZ point clouds in one viewer window.
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
	/// Vector of data streams containing clouds to be displayed.
	std::vector<Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr, Base::DataStreamBuffer::Newest,
			Base::Synchronization::Mutex> *> in_clouds;

	// Handlers
	Base::EventHandler2 h_on_spin;

	/// Vector of event handlers for refresing of individual clouds.
	std::vector<Base::EventHandler2*> handlers;
	
	// Handler functions.
	/// Refreshes n-th point cloud.
	void on_cloud_xyzN(int n);
	void on_spin();

	/// Point cloud viewer.
	pcl::visualization::PCLVisualizer * viewer;

	// Property enabling to change the name of displayed window.
	Base::Property<std::string> title;

	/// Number of clouds to display.
	Base::Property<int> count;

	/// Property for setting the colours of clouds. From default it will be set to 1 row with 255, 255, 255 (white cloud).
	Base::Property<cv::Mat, Types::MatrixTranslator> clouds_colours;
	
	Base::Property<bool> prop_coordinate_system;

};

} //: namespace XYZCloudViewer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("XYZCloudViewer", Processors::XYZCloudViewer::XYZCloudViewer)

#endif /* XYZCLOUDVIEWER_HPP_ */
