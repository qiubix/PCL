/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef SIFTADDER_HPP_
#define SIFTADDER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>
#include <vector>

namespace Processors {
namespace SIFTAdder {

/*!
 * \class SIFTAdder
 * \brief SIFTAdder processor class.
 *
 * SIFTAdder processor.
 */
class SIFTAdder: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SIFTAdder(const std::string & name = "SIFTAdder");

	/*!
	 * Destructor
	 */
	virtual ~SIFTAdder();

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
	
	bool matIsEqual(const cv::Mat mat1, const cv::Mat mat2);


// Input data streams

		Base::DataStreamIn<cv::Mat> in_descriptors;

// Output data streams

		Base::DataStreamOut<vector<cv::Mat> > out_descriptors;
	// Handlers
	Base::EventHandler2 h_add;

	
	// Handlers
	void add();
	
	vector<cv::Mat> descriptors;

};

} //: namespace SIFTAdder
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SIFTAdder", Processors::SIFTAdder::SIFTAdder)

#endif /* SIFTADDER_HPP_ */
