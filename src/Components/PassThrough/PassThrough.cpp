/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "PassThrough.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace PassThrough {

PassThrough::PassThrough(const std::string & name) :
		Base::Component(name) , 
		xa("x.a", 0), 
		xb("x.b", 0), 
		ya("y.a", 0), 
		yb("y.b", 0), 
		za("z.a", 0), 
		zb("z.b", 0), 
		negative("negative", false) {
		registerProperty(xa);
		registerProperty(xb);
		registerProperty(ya);
		registerProperty(yb);
		registerProperty(za);
		registerProperty(zb);
		registerProperty(negative);

}

PassThrough::~PassThrough() {
}

void PassThrough::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_cloud", &in_cloud);
registerStream("out_cloud", &out_cloud);
	// Register handlers
	h_filter.setup(boost::bind(&PassThrough::filter, this));
	registerHandler("filter", &h_filter);
	addDependency("filter", &in_cloud);

}

bool PassThrough::onInit() {

	return true;
}

bool PassThrough::onFinish() {
	return true;
}

bool PassThrough::onStop() {
	return true;
}

bool PassThrough::onStart() {
	return true;
}

void PassThrough::filter() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud.read();
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (xa, xb);
	pass.setFilterLimitsNegative (negative);
	pass.filter (*cloud);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (ya, yb);
	pass.setFilterLimitsNegative (negative);
	pass.filter (*cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (za, zb);
	pass.setFilterLimitsNegative (negative);
	pass.filter (*cloud);
	out_cloud.write(cloud);
}



} //: namespace PassThrough
} //: namespace Processors
