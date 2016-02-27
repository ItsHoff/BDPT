#pragma once


#include "RTTriangle.hpp"
#include "RaycastResult.hpp"
#include "rtlib.hpp"
#include "Bvh.hpp"

#include "base/String.hpp"

#include <vector>
#include <atomic>

namespace FW
{


// Given a vector n, forms an orthogonal matrix with n as the last column, i.e.,
// a coordinate system aligned such that n is its local z axis.
Mat3f formBasis(const Vec3f& n);

Vec2f getTexelCoords(Vec2f uv, const Vec2i size);

struct CheckNode { 
	Node* node;
	F32 t;
	CheckNode(Node* n, F32 t_n) :
		node(n), t(t_n) {}
};

#ifdef ISPC
struct ISPCCheckNode { 
	ISPCNode* node;
	F32 t;
	ISPCCheckNode(ISPCNode* n, F32 t_n) :
		node(n), t(t_n) {}
};
#endif


// Main class for tracing rays using BVHs.
class RayTracer {
public:
                        RayTracer				(void);
                        ~RayTracer				(void);

						void					constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode);

    void saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles);
    void loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles);

    RaycastResult raycast(const Vec3f& orig, const Vec3f& dir,std::vector<CheckNode>& b_nodes) const;
#ifdef ISPC
    RaycastResult ispcRaycast(const Vec3f & orig, const Vec3f & dir, std::vector<ISPCCheckNode>& b_nodes) const;
#endif // ISPC

    // This function computes an MD5 checksum of the input scene data,
    // WITH the assumption that all vertices are allocated in one big chunk.
    static FW::String	computeMD5				(const std::vector<Vec3f>& vertices);

    std::vector<RTTriangle>* m_triangles;
	BVH bvh;
	U32 max_size;

	void resetRayCounter() { m_rayCount = 0; }
	int getRayCount() { return m_rayCount; }

private:
	mutable std::atomic<int> m_rayCount;
};


} // namespace FW