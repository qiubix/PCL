/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "StatisticalOutlierRemoval.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/filters/statistical_outlier_removal.h>

namespace Processors {
namespace StatisticalOutlierRemoval {

StatisticalOutlierRemoval::StatisticalOutlierRemoval(const std::string & name) :
		Base::Component(name) , 
		negative("negative", false) {
		registerProperty(negative);

}

StatisticalOutlierRemoval::~StatisticalOutlierRemoval() {
}

void StatisticalOutlierRemoval::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	// Register handlers
	h_filter.setup(boost::bind(&StatisticalOutlierRemoval::filter, this));
	registerHandler("filter", &h_filter);
	addDependency("filter", &in_cloud_xyzrgb);

}

bool StatisticalOutlierRemoval::onInit() {

	return true;
}

bool StatisticalOutlierRemoval::onFinish() {
	return true;
}

bool StatisticalOutlierRemoval::onStop() {
	return true;
}

bool StatisticalOutlierRemoval::onStart() {
	return true;
}

void StatisticalOutlierRemoval::filter() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.setNegative (negative);
	sor.filter (*cloud);
	out_cloud_xyzrgb.write(cloud);
}



} //: namespace StatisticalOutlierRemoval
} //: namespace Processors
