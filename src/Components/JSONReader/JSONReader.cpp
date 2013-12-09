/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "JSONReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace JSONReader {

JSONReader::JSONReader(const std::string & name) :
		Base::Component(name) , 
		filename("filename", std::string("")) {
		registerProperty(filename);

}

JSONReader::~JSONReader() {
}

void JSONReader::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_cloud_xyzrgbsift", &out_cloud_xyzrgbsift);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	// Register handlers
	h_read.setup(boost::bind(&JSONReader::read, this));
	registerHandler("read", &h_read);
	addDependency("read", NULL);

}

bool JSONReader::onInit() {

	return true;
}

bool JSONReader::onFinish() {
	return true;
}

bool JSONReader::onStop() {
	return true;
}

bool JSONReader::onStart() {
	return true;
}

void JSONReader::read() {
	cout<<"JSONReader::read()"<<endl;
	//ptree ptree_file_rgb;
	//string filename_rgb = filename;
	//if(replace(filename_rgb, ".json", "_xyzrgb.json"))
		//try{
			//read_json(filename_rgb, ptree_file_rgb);
		//}
		//catch(std::exception const& e){
			//LOG(LERROR) << "JSONReader: file "<< filename_rgb <<" not found\n";
			//return;	
		//}
	//else{
		//LOG(LERROR) << "Nazwa pliku nie ma rozszerzenia .json \n";
	//}
	
	//ptree ptree_file_sift;
	//string filename_sift = filename;
	//if(replace(filename_sift, ".json", "_sift.json"))
		//try{
			//read_json(filename_sift, ptree_file_sift);
		//}
		//catch(std::exception const& e){
			//LOG(LERROR) << "JSONReader: file "<< filename_sift <<" not found\n";
			//return;	
		//}
		
		
	ptree ptree_file;
	try{
		read_json(filename, ptree_file);
		}
	catch(std::exception const& e){
		LOG(LERROR) << "JSONReader: file "<< filename <<" not found\n";
		return;	
		}		

	
	BOOST_FOREACH(boost::property_tree::ptree::value_type &v0, ptree_file){//clouds
		pcl::PointCloud<PointXYZRGBSIFT>::Ptr cloud_xyzrgbsift (new pcl::PointCloud<PointXYZRGBSIFT>());
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift (new pcl::PointCloud<PointXYZSIFT>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>());
		BOOST_FOREACH(boost::property_tree::ptree::value_type &v, v0.second)//points
			{
				
				pcl::PointXYZ point_xyz;
				pcl::PointXYZRGB point_xyzrgb;
				point_xyz.x = point_xyzrgb.x = boost::lexical_cast<float>(v.second.get_child("x").data());
				point_xyz.y = point_xyzrgb.y = boost::lexical_cast<float>(v.second.get_child("y").data());
				point_xyz.z = point_xyzrgb.z = boost::lexical_cast<float>(v.second.get_child("z").data());
				try{//color
					point_xyzrgb.r = boost::lexical_cast<float>(v.second.get_child("R").data());
					point_xyzrgb.g = boost::lexical_cast<float>(v.second.get_child("G").data());
					point_xyzrgb.b = boost::lexical_cast<float>(v.second.get_child("B").data());				
				}
				catch (std::exception const& e){
				}
				try{
					//SIFT
					PointXYZRGBSIFT point_xyzrgbsift;
					PointXYZSIFT point_xyzsift;
					float descriptor[128];
					int i=0;
					BOOST_FOREACH(boost::property_tree::ptree::value_type &v2, v.second.get_child("Descriptor")){
						//std::cout << v2.second.data() << " ";
						point_xyzsift.descriptor[i] = boost::lexical_cast<float>(v2.second.data());
						descriptor[i++] = boost::lexical_cast<float>(v2.second.data());
					}		
					cv::Mat d(1, 128, CV_32F, &descriptor);
					point_xyzrgbsift.descriptor=d;
					
					point_xyzsift.x = point_xyzrgbsift.x = boost::lexical_cast<float>(v.second.get_child("x").data());
					point_xyzsift.y = point_xyzrgbsift.y = boost::lexical_cast<float>(v.second.get_child("y").data());
					point_xyzsift.z = point_xyzrgbsift.z = boost::lexical_cast<float>(v.second.get_child("z").data());
					point_xyzrgbsift.r = boost::lexical_cast<float>(v.second.get_child("R").data());
					point_xyzrgbsift.g = boost::lexical_cast<float>(v.second.get_child("G").data());
					point_xyzrgbsift.b = boost::lexical_cast<float>(v.second.get_child("B").data());
					
					cloud_xyzrgbsift->push_back(point_xyzrgbsift);	
					cloud_xyzsift->push_back(point_xyzsift);					
									
				}
				catch (std::exception const& e){
					//std::cout<<"Brak deskryptora"<<std::endl;
				}
				
				cloud_xyz->push_back(point_xyz);
				cloud_xyzrgb->push_back(point_xyzrgb);
					//std::cout << v.second.data() << std::endl;
					// etc
				}
		out_cloud_xyzrgbsift.write(cloud_xyzrgbsift);
		out_cloud_xyzsift.write(cloud_xyzsift);
		out_cloud_xyz.write(cloud_xyz);
		out_cloud_xyzrgb.write(cloud_xyzrgb);
		//break;
	}	
		
	
	

}

bool JSONReader::replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

} //: namespace JSONReader
} //: namespace Processors
