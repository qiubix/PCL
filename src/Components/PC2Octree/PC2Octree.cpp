/*!
 * \file
 * \brief
 * \author Tomek Kornuta,,,
 */

#include <memory>
#include <string>

#include "PC2Octree.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

using namespace pcl::octree;

namespace Processors {
namespace PC2Octree {

PC2Octree::PC2Octree(const std::string & name) :
		Base::Component(name)  {

}

PC2Octree::~PC2Octree() {
}

void PC2Octree::prepareInterface() {
	// Register data streams.
	registerStream("in_cloud", &in_cloud_xyz);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	// Register handlers
	h_cloud_xyzrgb_to_octree.setup(boost::bind(&PC2Octree::cloud_xyzrgb_to_octree, this));
	registerHandler("cloud_xyzrgb_to_octree", &h_cloud_xyzrgb_to_octree);
	addDependency("cloud_xyzrgb_to_octree", &in_cloud_xyzrgb);


}

bool PC2Octree::onInit() {

	return true;
}

bool PC2Octree::onFinish() {
	return true;
}

bool PC2Octree::onStop() {
	return true;
}

bool PC2Octree::onStart() {
	return true;
}

void PC2Octree::cloud_xyzrgb_to_octree() {
	LOG(LTRACE) << "PC2Octree::cloud_xyzrgb_to_octree";
	// Read from dataport.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

	// Set voxel resolution.
	float voxelSize = 0.01f;
	pcl::octree::OctreePointCloud<pcl::PointXYZRGB, pcl::octree::OctreeContainerPointIndices> octree (voxelSize);
	// Set input cloud.
	octree.setInputCloud(cloud);
	// Calculate bounding box of input cloud.
	octree.defineBoundingBox();

	// Add points from input cloud to octree.
	octree.addPointsFromInputCloud ();

	//...?


  // breadth-first iterator test

  unsigned int lastDepth = 0;
  unsigned int branchNodeCount = 0;
  unsigned int leafNodeCount = 0;

  bool leafNodeVisited = false;

  pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::BreadthFirstIterator bfIt;
  const pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::BreadthFirstIterator bfIt_end = octree.breadth_end();

  for (bfIt = octree.breadth_begin(); bfIt != bfIt_end; ++bfIt)
  {
//    if (bfIt.getCurrentOctreeDepth () != lastDepth)
    	LOG(LWARNING) << "depth = " << bfIt.getCurrentOctreeDepth ();

    lastDepth = bfIt.getCurrentOctreeDepth ();
    

	unsigned char child_idx;
  // current node is a branch node
  // BranchNode* current_branch =   static_cast<BranchNode*> (FIFO_entry.node_);
  pcl::octree::OctreeNode* node = bfIt.getCurrentOctreeNode(); 
  if (node->getNodeType () == BRANCH_NODE) 
  {
	LOG(LWARNING) << "to jest branch";
//"  getLeafCount=" << node->getLeafCount() << " getBranchCount=" <<node->getBranchCount();
	OctreeBranchNode<pcl::PointXYZRGB>* branch_node =   static_cast<OctreeBranchNode<pcl::PointXYZRGB>*> (node);
	// iterate over all children
	for (child_idx = 0; child_idx < 8 ; ++child_idx)
	{
	
 		// if child exist
		if (branch_node->hasChild(child_idx))
		{
		LOG(LWARNING) << "ma dziecko "<<(int)child_idx;
 
//			BranchNode* current_branch = octree->getBranchChildPtr(*current_branch, child_idx);
		}
	}

//    if (bfIt.isBranchNode ())
      branchNodeCount++;
    }

	if (node->getNodeType () == LEAF_NODE) 
	{
		LOG(LWARNING) << "to jest leaf";
//		OctreeLeafNode<pcl::PointXYZRGB>* leaf_node =   static_cast<OctreeLeafNode<pcl::PointXYZRGB>*> (node);
		OctreeLeafNode< OctreeContainerPointIndices >* leaf_node =   static_cast< OctreeLeafNode<OctreeContainerPointIndices>* > (node);
		LOG(LWARNING) << "leaf_node->size = " << leaf_node->getContainer().getSize();

		std::vector<int> point_indices;
 		leaf_node->getContainer().getPointIndices(point_indices);
		//std::vector<int>::iterator it;
		for(unsigned int i=0; i<leaf_node->getContainer().getSize(); i++)
		{
			LOG(LWARNING) << "iteruję " << i << " index=" << point_indices[i];
///			octree.getPointByIndex(point_indices[i]);
			pcl::PointXYZRGB p = cloud->at(point_indices[i]);
			LOG(LWARNING) << "p.x = " << p.x << "p.y = " << p.y << "p.z = " << p.z;
			
			
		}		
		
//		LOG(LWARNING) << "leaf_node->size = " << leaf_node->getContainer().getSize();


//		pcl::octree::OctreeContainerPointIndices  cont;
//		LOG(LWARNING) << "cont.size = " << cont.getSize();

//pcl::octree::OctreeContainerPointIndices  cont2 = static_cast<pcl::octree::OctreeContainerPointIndices> (leaf_node->getContainer());

//		pcl::PointXYZRGB p = leaf_node->getContainer();
//		LOG(LWARNING) << "p.x = " << p.x << "p.y = " << p.y << "p.z = " << p.z;
/*for (child_idx = 0; child_idx < 8 ; ++child_idx)
	{
		if (leaf_node->hasChild(child_idx))
		{
			LOG(LWARNING) << "ma dziecko "<<(int)child_idx;
		}
}*/
/*		LOG(LWARNING) << " pcl::PointXYZRGB -> x = " << (*leaf_node)->x 
			<<" y = " << (*leaf_node)->y
			<<" z = " << (*leaf_node)->z;*/

		leafNodeCount++;
	}

/*    if (bfIt.isLeafNode ())
    {
      leafNodeCount++;
      leafNodeVisited = true;
    }
*/
  }

LOG(LWARNING) << "ELO! branchNodeCount: " << branchNodeCount;
LOG(LWARNING) << "ELO! leafNodeCount: " << leafNodeCount;



  // instantiate iterator for octree
       pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::LeafNodeIterator it;// (octree);
	it = octree.leaf_begin();

     std::vector<int> indexVector;

     while (*++it)
     {
//       it.getData (indexVector);
	//pcl::octree::OctreeLeafContainer<pcl::PointXYZRGB> c = it.getLeafContainer();
     } 

	// Delete octree data structure (pushes allocated nodes to memory pool!)
	// octree.deleteTree ();
}


} //: namespace PC2Octree
} //: namespace Processors
