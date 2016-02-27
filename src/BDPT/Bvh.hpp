#pragma once

#ifdef ISPC
	#define TRIS_IN_LEAF 8
#else
	#define TRIS_IN_LEAF 4
#endif // ISPC

#include <vector>

#include "BvhNode.hpp"
#include "RTTriangle.hpp"
#ifdef ISPC
	#include "ispc_util.hpp"
#endif

namespace FW
{ namespace BDPT
{

// Defines the split modes available for BVH construction
enum SplitMode {
    SplitMode_SpatialMedian,
    SplitMode_ObjectMedian,
    SplitMode_Sah,
    SplitMode_None
};

class BVH {
 public:
    BVH() {}
    ~BVH() {}

	std::vector<Node> hierarchy_;           
	std::vector<U32> tri_indices_;          // Triangle indices for sorting the triangles and saving the hierarchy
    std::vector<RTTriangle>* triangles_;    // Triangles for the intersections
#ifdef ISPC
	std::vector<ISPCNode> ispc_hierarchy_;
	std::vector<ispc::AABB> ispc_bbs_;              // AABBs for ispc intersections
	std::vector<ispc::Triangle> ispc_triangles_;    // Triangles for ispc intersections
#endif

    // Construct hierarchy with no splitting
	void construct(std::vector<RTTriangle>& triangles);
    // Construct hierarchy with object median splitting
	void constructObjectMedian(std::vector<RTTriangle>& triangles);
    // Construct hierarchy with spatial median splitting
	void constructSpatialMedian(std::vector<RTTriangle>& triangles);
    // Construct hierarchy with SAH splitting
	void constructSAH(std::vector<RTTriangle>& triangles);
    // Sort triangles based on tri_indices
	void sortTriangles();
#ifdef ISPC
	// Convert the binary tree to tree with 2^power splits
	void rebranchForISPC(U32 power);	
#endif

 private:
	const U32 tris_in_leaf = TRIS_IN_LEAF;      // Maximum amount of triangles in a leaf node

    // Compute the bounding box of triangles in [start, end)
	AABB computeBoundingBox(const U32 start, const U32 end) const;
    // Compute the bounding box of the existing bb and triangles in [start, end) 
	AABB computeIncreasedBB(const AABB& bb, const U32 start, const U32 end) const;
    // Find the longest axis of the bounding_box for splitting
	U32 chooseLongestAxis(const AABB& bounding_box) const;
    // Find the spatial median split index for node. Assumes that the tri_indices list has been sorted.
	U32 findSpatialMedian(const Node& node, const U32 split_axis) const;
    // Calculate the SAH for n_l nodes in left_bb and n_r nodes in right_bb
	F32 calculateSAH(const AABB& left_bb, const U32 n_l, const AABB& right_bb, const U32 n_r) const;
    // create a new node for triangles in [start, end)
	Node createNode(const U32 start, const U32 end) const;
};

} // namespace BDPT
} // namespace FW