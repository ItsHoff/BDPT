#pragma once

#include <atomic>
#include <vector>

#include "base/String.hpp"

#include "Bvh.hpp"
#include "RaycastResult.hpp"
#include "RTTriangle.hpp"

namespace FW
{ namespace BDPT 
{

// Given a vector n, forms an orthogonal matrix with n as the last column, 
// i.e. a coordinate system aligned such that n is its local z axis.
Mat3f formBasis(const Vec3f& n);

// Get texel indices of texel nearest to the uv vector.
Vec2f getTexelCoords(Vec2f uv, const Vec2i size);

// Helper struct for the hierarchy traversal
struct CheckNode { 
	Node* node;     // Pointer to the corresponding node
	F32 t;          // t value of the node-ray intersection

	CheckNode(Node* n, F32 t_n) :
		node(n), t(t_n) {
    }
};

#ifdef ISPC
struct ISPCCheckNode { 
	ISPCNode* node;
	F32 t;

	ISPCCheckNode(ISPCNode* n, F32 t_n) :
		node(n), t(t_n) {
    }
};
#endif


// Main class for tracing rays using BVHs.
class RayTracer {
 public:
    std::vector<RTTriangle>* m_triangles;
	BVH m_bvh;

    RayTracer() {}
    ~RayTracer() {};

	void constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode split_mode);
    void saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles);
    void loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles);
    // Trace the given ray. b_nodes is a pre-allocated buffer for traversal stack
    RaycastResult raycast(const Vec3f& orig, const Vec3f& dir, std::vector<CheckNode>& b_nodes) const;
#ifdef ISPC
    RaycastResult ispcRaycast(const Vec3f & orig, const Vec3f & dir, std::vector<ISPCCheckNode>& b_nodes) const;
#endif
    // This function computes an MD5 checksum of the input scene data,
    // WITH the assumption that all vertices are allocated in one big chunk.
    static FW::String computeMD5(const std::vector<Vec3f>& vertices);
	void resetRayCounter() { m_rayCount = 0; }
	int getRayCount() { return m_rayCount; }

 private:
	mutable std::atomic<int> m_rayCount;
};

} // namespace BDPT
} // namespace FW