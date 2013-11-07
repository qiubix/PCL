/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "SIFTAdder.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <fstream>

namespace Processors {
namespace SIFTAdder {

SIFTAdder::SIFTAdder(const std::string & name) :
		Base::Component(name)  {

}

SIFTAdder::~SIFTAdder() {
}

void SIFTAdder::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_descriptors", &in_descriptors);
registerStream("out_descriptors", &out_descriptors);
	// Register handlers
	h_add.setup(boost::bind(&SIFTAdder::add, this));
	registerHandler("add", &h_add);
	addDependency("add", &in_descriptors);

}

bool SIFTAdder::onInit() {
	
	return true;
}

bool SIFTAdder::onFinish() {
	return true;
}

bool SIFTAdder::onStop() {
	std::fstream plik;
	plik.open( "/home/mlaszkow/test.txt", std::ios::out );
	plik<<"Deskryptory:"<<endl;
	for (int i = 0; i< descriptors.size(); i++)
		plik<<descriptors[i]<<endl;
	plik.close();
	return true;
}

bool SIFTAdder::onStart() {
	return true;
}

void SIFTAdder::add() {
	cv::Mat new_descriptors = in_descriptors.read();

	for(int i = 0; i<new_descriptors.rows; i++){
		bool unique = true;
		for(int j = 0; j<descriptors.size(); j++){
			if(matIsEqual(descriptors[j],new_descriptors.row(i))){
				unique = false;
				break;
			}
		}
		if (unique)
			descriptors.push_back(new_descriptors.row(i));
			
		}
}



 bool SIFTAdder::matIsEqual(const cv::Mat mat1, const cv::Mat mat2){
    // treat two empty mat as identical as well
    if (mat1.empty() && mat2.empty()) {
        return true;
    }
    // if dimensionality of two mat is not identical, these two mat is not identical
    if (mat1.cols != mat2.cols || mat1.rows != mat2.rows || mat1.dims != mat2.dims) {
        return false;
    }
    cv::Mat diff;
    cv::compare(mat1, mat2, diff, cv::CMP_NE);
    int nz = cv::countNonZero(diff);
    return nz==0;
}

} //: namespace SIFTAdder
} //: namespace Processors
