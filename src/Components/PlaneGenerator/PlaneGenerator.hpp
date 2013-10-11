/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef PLANEGENERATOR_HPP_
#define PLANEGENERATOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Processors {
namespace PlaneGenerator {

/*!
 * \class PlaneGenerator
 * \brief PlaneGenerator processor class.
 *
 * PlaneGenerator processor.
 */
class PlaneGenerator: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	PlaneGenerator(const std::string & name = "PlaneGenerator");

	/*!
	 * Destructor
	 */
	virtual ~PlaneGenerator();

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


// Output data streams

		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_pcl_ptr;
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ> > out_pcl;
	// Handlers
	Base::EventHandler2 h_Generate;

	
	// Handlers
	void Generate();
	
	
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;

};

} //: namespace PlaneGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("PlaneGenerator", Processors::PlaneGenerator::PlaneGenerator)

#endif /* PLANEGENERATOR_HPP_ */
