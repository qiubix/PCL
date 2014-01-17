/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef SOMPCDREADER_HPP_
#define SOMPCDREADER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/SIFTObjectModel.hpp> 
#include <Types/SIFTObjectModelFactory.hpp> 
#include <Types/PointXYZSIFT.hpp> 

namespace Processors {
namespace SOMPCDReader {

/*!
 * \class SIFTObjectModelFactory
 * \brief SIFTObjectModelFactory processor class.
 *
 * SIFTObjectModelFactory processor.
 */
class SOMPCDReader: public Base::Component,SIFTObjectModelFactory { ///: SIFTObjectModelFactory
public:
	/*!
	 * Constructor.
	 */
	SOMPCDReader(const std::string & name = "SOMPCDReader");

	/*!
	 * Destructor
	 */
	virtual ~SOMPCDReader();

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

    Base::DataStreamOut<std::vector<AbstractObject*> > out_models;
	// Handlers
//	Base::EventHandler2 h_produce;
	Base::EventHandler2 h_loadModels;
		Base::Property<string> names;

	// Handlers

	void loadModels();
	

};

} //: namespace SIFTObjectModelFactory
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SOMPCDReader", Processors::SOMPCDReader::SOMPCDReader)

#endif /* SOMPCDReader_HPP_ */
