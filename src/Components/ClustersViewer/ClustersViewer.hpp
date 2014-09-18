/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef CLUSTERSVIEWER_HPP_
#define CLUSTERSVIEWER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/visualization/pcl_visualizer.h>

namespace Processors {
namespace ClustersViewer {

/*!
 * \class ClustersViewer
 * \brief ClustersViewer processor class.
 *
 * ClustersViewer processor.
 */
class ClustersViewer: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ClustersViewer(const std::string & name = "ClustersViewer");

	/*!
	 * Destructor
	 */
	virtual ~ClustersViewer();

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
	Base::DataStreamIn<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > in_clouds;		

	// Handlers
	Base::EventHandler2 h_on_clouds;
	Base::EventHandler2 h_on_spin;

	// Handlers
	void on_clouds();
	void on_spin();
	
	// Property enabling to change the name of displayed window.
	Base::Property<std::string> title;
	Base::Property<bool> prop_coordinate_system;
	
	/// Point cloud viewer.
	pcl::visualization::PCLVisualizer * viewer;
	
	
	int count;

};

} //: namespace ClustersViewer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ClustersViewer", Processors::ClustersViewer::ClustersViewer)

#endif /* CLUSTERSVIEWER_HPP_ */
