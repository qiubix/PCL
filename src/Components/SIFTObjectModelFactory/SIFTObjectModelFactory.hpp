/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef SIFTOBJECTMODELFACTORY_HPP_
#define SIFTOBJECTMODELFACTORY_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/SIFTObjectModel.hpp> 
#include <Types/PointXYZSIFT.hpp> 

namespace Processors {
namespace SIFTObjectModelFactory {

/*!
 * \class SIFTObjectModelFactory
 * \brief SIFTObjectModelFactory processor class.
 *
 * SIFTObjectModelFactory processor.
 */
class SIFTObjectModelFactory: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SIFTObjectModelFactory(const std::string & name = "SIFTObjectModelFactory");

	/*!
	 * Destructor
	 */
	virtual ~SIFTObjectModelFactory();

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

		Base::DataStreamOut<Types::SIFTObjectModel> out_model;
	// Handlers
	Base::EventHandler2 h_produce;
	Base::EventHandler2 h_loadModels;
		Base::Property<string> names;

	
	// Handlers
	void produce();
	void loadModels();

};

} //: namespace SIFTObjectModelFactory
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SIFTObjectModelFactory", Processors::SIFTObjectModelFactory::SIFTObjectModelFactory)

#endif /* SIFTOBJECTMODELFACTORY_HPP_ */
