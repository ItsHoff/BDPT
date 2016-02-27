#pragma once

#ifdef ISPC
	#define TRIS_IN_LEAF 8
#else
	#define TRIS_IN_LEAF 4
#endif // ISPC


#include "BvhNode.hpp"

#include <vector>

#ifdef ISPC
	#include "ispc_util.hpp"
#endif
#include "RTTriangle.hpp"

namespace FW
{ namespace BDPT
{

enum SplitMode {
    SplitMode_SpatialMedian,
    SplitMode_ObjectMedian,
    SplitMode_Sah,
    SplitMode_None,
    SplitMode_Linear
};

class BVH
{
public:
	BVH();
	~BVH();

	std::vector<Node> hierarchy_;
	std::vector<U32> tri_indices_;
    std::vector<RTTriangle>* triangles_;
#ifdef ISPC
	std::vector<ISPCNode> ispc_hierarchy_;
	std::vector<ispc::AABB> ispc_bbs_;
	std::vector<ispc::Triangle> ispc_triangles_;
#endif

	void construct(std::vector<RTTriangle>& triangles);
	void constructObjectMedian(std::vector<RTTriangle>& triangles);
	void constructSpatialMedian(std::vector<RTTriangle>& triangles);
	void constructSAH(std::vector<RTTriangle>& triangles);
	void sortTriangles();
#ifdef ISPC
	// Convert the binary tree to tree with 2^power splits
	void rebranchForISPC(U32 power);	
#endif

private:
	const U32 tris_in_leaf = TRIS_IN_LEAF;

	AABB computeBoundingBox(const U32 start, const U32 end) const;
	AABB computeIncreasedBB(const AABB& bb, const U32 start, const U32 end) const;
	U32 chooseLongestAxis(const AABB& bounding_box) const;
	U32 findSpatialMedian(const Node& node, const U32 split_axis) const;
	F32 calculateSAH(const AABB& left_bb, const U32 n_l, const AABB& right_bb, const U32 n_r) const;
	Node createNode(const U32 start, const U32 end) const;
};

} // namespace BDPT
} // namespace FW