#include "Bvh.hpp"

#include <algorithm>
#include <deque>

namespace FW
{ namespace BDPT
{

void BVH::construct(std::vector<RTTriangle>& triangles) {
    // Initialize triangle vectors
	triangles_ = &triangles;
	tri_indices_.reserve(triangles.size());
	for (int i = 0; i < triangles.size(); i++) {
		tri_indices_.push_back(i);
	}
    // Simply add a root node and sort the triangles
	hierarchy_.push_back(createNode(0, triangles.size()));
	U32 split_axis = chooseLongestAxis(hierarchy_[0].bb);
	auto sort_f = [&](U32 t1, U32 t2) -> bool {
		Vec3f c1 = (*triangles_)[t1].center;
		Vec3f c2 = (*triangles_)[t2].center;
		return c1[split_axis] < c2[split_axis];
	};
	std::sort(tri_indices_.begin(), tri_indices_.end(), sort_f);
    // Sort the main triangle vector for performance
	sortTriangles();
}

void BVH::constructObjectMedian(std::vector<RTTriangle>& triangles) {
    // Initialize triangle vectors
	triangles_ = &triangles;
	tri_indices_.reserve(triangles.size());
	for (int i = 0; i < triangles.size(); i++) {
		tri_indices_.push_back(i);
	}
	hierarchy_.push_back(createNode(0, triangles.size()));
	std::vector<U32> nodes_to_split = {0};      // Stack for nodes still to be split
	nodes_to_split.reserve(log2f(triangles.size()));
	while (!nodes_to_split.empty()) {
		Node& node_to_split = hierarchy_[nodes_to_split.back()];
		nodes_to_split.pop_back();
		U32 start = node_to_split.start;
		U32 end = node_to_split.end;
        // Choose the longest axis for splitting and sort the triangles 
		U32 split_axis = chooseLongestAxis(node_to_split.bb);
		auto comp_f = [&](U32 t1, U32 t2) -> bool {
			Vec3f c1 = (*triangles_)[t1].center;
			Vec3f c2 = (*triangles_)[t2].center;
			return c1[split_axis] < c2[split_axis];
		};
		std::sort(tri_indices_.begin() + start, tri_indices_.begin() + end, comp_f);
        // Find the middle index and add the child nodes to the split stack 
        // if they are still too big.
		U32 mid = (start + end) / 2;
		node_to_split.left_child = hierarchy_.size();
		node_to_split.right_child = hierarchy_.size() + 1;
		if (mid - start > tris_in_leaf) {
			nodes_to_split.push_back(node_to_split.left_child);
		}
		if (end - mid > tris_in_leaf) {
			nodes_to_split.push_back(node_to_split.right_child);
		}
		hierarchy_.push_back(createNode(start, mid));
		hierarchy_.push_back(createNode(mid, end));
	}
    // Sort the main triangle vector for performance
	sortTriangles();
}

void BVH::constructSpatialMedian(std::vector<RTTriangle>& triangles) {
    // Initialize triangle vectors
	triangles_ = &triangles;
	tri_indices_.reserve(triangles.size());
	for (int i = 0; i < triangles.size(); i++) {
		tri_indices_.push_back(i);
	}
	hierarchy_.push_back(createNode(0, triangles.size()));
	std::vector<U32> nodes_to_split = {0};      // Stack for nodes still to be split
	nodes_to_split.reserve(log2f(triangles.size()));
	while (!nodes_to_split.empty()) {
		Node& node_to_split = hierarchy_[nodes_to_split.back()];
		nodes_to_split.pop_back();
		U32 start = node_to_split.start;
		U32 end = node_to_split.end;
        // Choose the longest axis for splitting and sort the triangles 
		U32 split_axis = chooseLongestAxis(node_to_split.bb);
		auto sort_f = [&](U32 t1, U32 t2) -> bool {
			Vec3f c1 = (*triangles_)[t1].center;
			Vec3f c2 = (*triangles_)[t2].center;
			return c1[split_axis] < c2[split_axis];
		};
		std::sort(tri_indices_.begin() + start, tri_indices_.begin() + end, sort_f);
        // Find the split index
		U32 mid_i = findSpatialMedian(node_to_split, split_axis);
        // If all triangles fall to one side of the spatial median
        // use object median instead
		if (mid_i == start || mid_i == end) {
			mid_i = (start + end) / 2;
		}
		node_to_split.left_child = hierarchy_.size();
		node_to_split.right_child = hierarchy_.size() + 1;
        // Add the childs to the split stack if they are still too big
		if (mid_i - start > tris_in_leaf) {
			nodes_to_split.push_back(node_to_split.left_child);
		}
		if (end - mid_i > tris_in_leaf) {
			nodes_to_split.push_back(node_to_split.right_child);
		}
        // Construct the child nodes
		hierarchy_.push_back(createNode(start, mid_i));
		hierarchy_.push_back(createNode(mid_i, end));
	}
    // Sort the main triangle vector for performance
	sortTriangles();
}

void BVH::constructSAH(std::vector<RTTriangle>& triangles) {
    // Initialize triangle vectors
	triangles_ = &triangles;
	tri_indices_.reserve(triangles.size());
	for (int i = 0; i < triangles.size(); i++) {
		tri_indices_.push_back(i);
	}
	hierarchy_.push_back(createNode(0, triangles.size()));
	std::vector<U32> nodes_to_split = {0};      // Stack for nodes still to be split
	nodes_to_split.reserve(log2f(triangles.size()));
	while (!nodes_to_split.empty()) {
		Node& node = hierarchy_[nodes_to_split.back()];
		nodes_to_split.pop_back();
		U32 start = node.start;
		U32 end = node.end;

        // Find the optimal split by SAH
		U32 min_axis, mid_i;
        F32 min_sah = FLT_MAX;
		for (U32 split_axis = 0; split_axis < 3; split_axis++) {
            // Sort the triangles on the current axis
			auto sort_f = [&](U32 t1, U32 t2) -> bool {
				Vec3f c1 = (*triangles_)[t1].center;
				Vec3f c2 = (*triangles_)[t2].center;
				return c1[split_axis] < c2[split_axis];
			};
			std::sort(tri_indices_.begin() + start, tri_indices_.begin() + end, sort_f);

            // Test split only between [start+1, end-1) because we want to keep splitting
            // nodes until tris_in_leaf for ispc
			U32 test_start = start + 1;
			U32 test_end =  end - 1;
			U32 test_length = test_end - test_start;
			U32 tests =  test_length + 1;
            // Incrementally compute all the necessary bounding boxes
			std::vector<std::pair<AABB, U32>> left_bbs;
			std::vector<std::pair<AABB, U32>> right_bbs;
			left_bbs.reserve(tests);
			right_bbs.reserve(tests);
			left_bbs.emplace_back(computeBoundingBox(start, test_start), test_start);
			right_bbs.emplace_back(computeBoundingBox(test_end, end), test_end);
			for (U32 i = 1; i < tests; i++) {
				F32 frac = 1.0 * i / (tests - 1);
				U32 left_i = test_start + frac * test_length;
				U32 right_i = test_end - frac * test_length;
				left_bbs.emplace_back(computeIncreasedBB(left_bbs[i-1].first, left_bbs[i-1].second, left_i), left_i);
				right_bbs.emplace_back(computeIncreasedBB(right_bbs[i-1].first, right_i, right_bbs[i-1].second), right_i);
			}
            // Calculate the SAH for all the possible splits and find the optimal one
			for (U32 i = 0; i < tests; i++) {
				AABB left_bb = left_bbs[i].first;
				AABB right_bb = right_bbs[tests - 1 - i].first;
				U32 split_i = left_bbs[i].second;
				F32 sah = calculateSAH(left_bb, split_i - start, right_bb, end - split_i);
				if (sah < min_sah) {
					min_axis = split_axis;
					mid_i = split_i;
					min_sah = sah;
				}
			}
		}

        // If the split axis is not the last tested we need to re-sort triangles
		if (min_axis != 2) {
			auto sort_f = [&](U32 t1, U32 t2) -> bool {
				Vec3f c1 = (*triangles_)[t1].center;
				Vec3f c2 = (*triangles_)[t2].center;
				return c1[min_axis] < c2[min_axis];
			};
			std::sort(tri_indices_.begin() + start, tri_indices_.begin() + end, sort_f);
		}

        // Finally construct the child nodes and add them to split stack 
        // if they are still too big
        node.left_child = hierarchy_.size();
        node.right_child = hierarchy_.size() + 1;
        if (mid_i - start > tris_in_leaf) {
            nodes_to_split.push_back(node.left_child);
        }
        if (end - mid_i > tris_in_leaf) {
            nodes_to_split.push_back(node.right_child);
        }
        hierarchy_.push_back(createNode(start, mid_i));
        hierarchy_.push_back(createNode(mid_i, end));
	}
    // Sort the main triangle vector for performance
	sortTriangles();
}

AABB BVH::computeBoundingBox(const U32 start, const U32 end) const {
    // Find the maximum and minimum coordinates for triangles between [start, end)
	Vec3f max = (*triangles_)[tri_indices_[start]].max();
	Vec3f min = (*triangles_)[tri_indices_[start]].min();
	for (U32 i = start+1; i < end; i++){
		max = FW::max(max, (*triangles_)[tri_indices_[i]].max());
		min = FW::min(min, (*triangles_)[tri_indices_[i]].min());
	}
	return AABB(min, max);
}

AABB BVH::computeIncreasedBB(const AABB& bb, const U32 start, const U32 end) const {
    // Find the maximum and minimum coordinates for existing bb and triangles between [start, end)
	Vec3f max = bb.max;
	Vec3f min = bb.min;
	for (U32 i = start; i < end; i++){
		max = FW::max(max, (*triangles_)[tri_indices_[i]].max());
		min = FW::min(min, (*triangles_)[tri_indices_[i]].min());
	}
	return AABB(min, max);
}

U32 BVH::chooseLongestAxis(const AABB& bounding_box) const {
    // Check all the axis and see which is the longest
	Vec3f length = bounding_box.max - bounding_box.min;
	if (length.z > length.y && length.z > length.x) {
		return 2;
	}
	else if (length.y > length.z && length.y > length.x) {
		return 1;
	}
	else {
		return 0;
	}
}

U32 BVH::findSpatialMedian(const Node& node, const U32 split_axis) const {
    // Calculate spatial mid point of the node on the split_axis
	F32 mid = (node.bb.max[split_axis] + node.bb.min[split_axis]) / 2;
	auto comp_f = [&](F32 mid, U32 t) -> bool {
		Vec3f c = (*triangles_)[t].center;
		return mid < c[split_axis];
	};
    // Find the first tri that is on the other side of the mid point
    // Assumes allready sorted tri_indices!
	U32 mid_i = std::upper_bound(tri_indices_.begin() + node.start, tri_indices_.begin() + node.end, mid, comp_f) - tri_indices_.begin();
	return mid_i;
}

F32 BVH::calculateSAH(const AABB& left_bb, const U32 n_l, const AABB& right_bb, const U32 n_r) const {
	F32 left_area = left_bb.area();
	F32 right_area = right_bb.area();
	return left_area*n_l + right_area*n_r;
}

Node BVH::createNode(const U32 start, const U32 end) const {
	Node node;
	node.bb = computeBoundingBox(start, end);
	node.start = start;
	node.end = end;
	node.left_child = -1;
	node.right_child = -1;
	return node;
}

void BVH::sortTriangles() {
	std::vector<RTTriangle> sorted_tris;
	sorted_tris.reserve(triangles_->size());
	for (S32 i : tri_indices_) {
		sorted_tris.push_back((*triangles_)[i]);
	}
	*triangles_ = sorted_tris;
}

#ifdef ISPC
void BVH::rebranchForISPC(U32 power) {
	// Convert triangles for ispc
	ispc_triangles_.reserve(triangles_->size());
	for (RTTriangle& triangle : *triangles_) {
		ispc_triangles_.push_back(convertToISPC(triangle));
	}

    // Add empty node first. It will be filled later.
	ispc_hierarchy_.emplace_back();
	ispc_bbs_.push_back(convertToISPC(hierarchy_[0].bb));
	std::vector<std::pair<U32, U32>> nodes_to_check;  // pair<index, ispc_index>
	nodes_to_check.emplace_back(0, 0);
	while (!nodes_to_check.empty()) {
		const std::pair<U32, U32> indices = nodes_to_check.back();
		nodes_to_check.pop_back();
		// Find all the children power levels down for each node (ie. up to 2^power)
		std::vector<U32> parents;
		std::vector<U32> children;
		parents.reserve(1 << power);
		children.reserve(1 << power);
		children.push_back(indices.first);
		for (U32 depth = 0; depth < power; depth++) {
			parents = children;
			children.clear();
			for (U32 parent : parents) {
				if (hierarchy_[parent].left_child != -1) {
					children.push_back(hierarchy_[parent].left_child);
					children.push_back(hierarchy_[parent].right_child);
                } else if (parent != indices.first) {
                    // If the node doesn't have children and isn't the current node
                    // we need to add it back to children to retain the branch.
                    children.push_back(parent);
                }
			}
		}

		// Fill the ispc information
		const Node& node = hierarchy_[indices.first];
		ISPCNode& ispc_node = ispc_hierarchy_[indices.second];
		ispc_node.n_children = children.size();
		ispc_node.first_child = ispc_hierarchy_.size();
		ispc_node.first_bb = ispc_bbs_.size();
		ispc_node.n_triangles = node.end - node.start;
		ispc_node.first_triangle = node.start;

		// Add children to hierarchy
		for (U32 child : children) {
			nodes_to_check.emplace_back(child, ispc_hierarchy_.size());
			ispc_hierarchy_.emplace_back();
			ispc_bbs_.push_back(convertToISPC(hierarchy_[child].bb));
		}
	}
}
#endif // ISPC
	
} //namespace BDPT
} //namespace FW