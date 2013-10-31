/*!
 * \file
 * \brief
 * \author Maciej Stefańczyk [maciek.slon@gmail.com]
 */

#include <memory>
#include <string>

#include "DepthConverter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/io/pcd_io.h>

namespace Processors {
namespace DepthConverter {

DepthConverter::DepthConverter(const std::string & name) :
		Base::Component(name)  {

}

DepthConverter::~DepthConverter() {
}

void DepthConverter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register data streams
	registerStream("in_depth", &in_depth);
	registerStream("in_color", &in_color);
	registerStream("in_mask", &in_mask);
	registerStream("in_camera_info", &in_camera_info);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyz_masked", &out_cloud_xyz_masked);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);

	// Register handlers
	h_process_depth.setup(boost::bind(&DepthConverter::process_depth, this));
	registerHandler("process_depth", &h_process_depth);
	//addDependency("process_depth", &in_depth);
	//addDependency("process_depth", &in_camera_info);
	h_process_depth_mask.setup(boost::bind(&DepthConverter::process_depth_mask, this));
	registerHandler("process_depth_mask", &h_process_depth_mask);
	addDependency("process_depth_mask", &in_depth);
	addDependency("process_depth_mask", &in_camera_info);	
	addDependency("process_depth_mask", &in_mask);
	h_process_all.setup(boost::bind(&DepthConverter::process_all, this));
	registerHandler("process_all", &h_process_all);
	addDependency("process_all", &in_depth);
	addDependency("process_all", &in_color);
	addDependency("process_all", &in_camera_info);

}

bool DepthConverter::onInit() {

	return true;
}

bool DepthConverter::onFinish() {
	return true;
}

bool DepthConverter::onStop() {
	return true;
}

bool DepthConverter::onStart() {
	return true;
}

void DepthConverter::process_depth() {
	Types::CameraInfo camera_info = in_camera_info.read();
	cv::Mat depth = in_depth.read();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(camera_info.width(), camera_info.height()));

//	cloud.width = camera_info.width();
//	cloud.height = camera_info.height();
//	cloud.points.resize(cloud.width * cloud.height);

	double fx_d = 0.001 / camera_info.fx();
	double fy_d = 0.001 / camera_info.fy();
	double cx_d = camera_info.cx();
	double cy_d = camera_info.cy();

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud->begin();
	const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth.data[0]);

	int row_step = depth.step1();
	for (int v = 0; v < (int) cloud->height; ++v, depth_row += row_step) {
		for (int u = 0; u < (int) cloud->width; ++u) {
			pcl::PointXYZ& pt = *pt_iter++;
			uint16_t depth = depth_row[u];

			// Missing points denoted by NaNs
			if (depth == 0) {
				pt.x = pt.y = pt.z = bad_point;
				continue;
			}

			// Fill in XYZ
			pt.x = (u - cx_d) * depth * fx_d;
			pt.y = (v - cy_d) * depth * fy_d;
			pt.z = depth * 0.001;
		}
	}

	out_cloud_xyz.write(cloud);
	/*pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
	CLOG(LNOTICE) << "Saved " << cloud->points.size () << " data points to test_pcd.pcd.";*/
}

void DepthConverter::process_depth_mask() {
	Types::CameraInfo camera_info = in_camera_info.read();
	cv::Mat depth = in_depth.read();
	cv::Mat mask = in_mask.read();
	mask.convertTo(mask, CV_32F);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(camera_info.width(), camera_info.height()));

//	cloud.width = camera_info.width();
//	cloud.height = camera_info.height();
//	cloud.points.resize(cloud.width * cloud.height);

	double fx_d = 0.001 / camera_info.fx();
	double fy_d = 0.001 / camera_info.fy();
	double cx_d = camera_info.cx();
	double cy_d = camera_info.cy();

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud->begin();
	const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth.data[0]);

	int row_step = depth.step1();
	for (int v = 0; v < (int) cloud->height; ++v, depth_row += row_step) {
		for (int u = 0; u < (int) cloud->width; ++u) {
			pcl::PointXYZ& pt = *pt_iter++;
			uint16_t depth = depth_row[u];

			// Missing points denoted by NaNs
			if (depth == 0 || mask.at<float>(v, u)==0) {
				pt.x = pt.y = pt.z = bad_point;
				continue;
			}

			// Fill in XYZ
			pt.x = (u - cx_d) * depth * fx_d;
			pt.y = (v - cy_d) * depth * fy_d;
			pt.z = depth * 0.001;
		}
	}

	out_cloud_xyz_masked.write(cloud);
	/*pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
	CLOG(LNOTICE) << "Saved " << cloud->points.size () << " data points to test_pcd.pcd.";*/
}

void DepthConverter::process_all() {
}



} //: namespace DepthConverter
} //: namespace Processors
