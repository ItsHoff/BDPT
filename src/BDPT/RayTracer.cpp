#define _CRT_SECURE_NO_WARNINGS

#include <fstream>
#include <queue>
#include <stdio.h>
#include <utility>

#include "base/Defs.hpp"
#include "base/Math.hpp"

#include "RayTracer.hpp"
#ifdef ISPC
	#include "intersect_ispc.h"
#endif


// Helper function for hashing scene data for caching BVHs
extern "C" void MD5Buffer( void* buffer, size_t bufLen, unsigned int* pDigest );

namespace FW
{ namespace BDPT
{

Vec2f getTexelCoords(Vec2f uv, const Vec2i size) {
	// uv should be mapped from [0,1]x[0,1] to the size of the image [0,X]x[0,Y] and
	// wrapped so that uv values beyond 1 are mapped back to [0,1] in a repeating manner
	// (so 0.5, 1.5, 2.5,... all map to 0.5)
	auto wrapped_uv = Vec2f(uv.x - floor(uv.x), uv.y - floor(uv.y));
	auto coords = Vec2f(wrapped_uv.x * size.x, wrapped_uv.y * size.y);
	return coords;
}

Mat3f formBasis(const Vec3f& n) {
	Vec3f q(n);
	Vec3f abs = n.abs();
	F32 min = abs.min();
    // Get a vector that is not parallel with n
	for (int i = 0; i < 3; i++){
		if (abs[i] == min){
			q[i] = 1;
			break;
		}
	}
    // Calculate the basis from cross product of the normal 
    // and the new disturbed normal
	Vec3f t = q.cross(n).normalized();
	Vec3f b = t.cross(n).normalized();
	Mat3f m;
	m.setCol(0, t);
	m.setCol(1, b);
	m.setCol(2, n);
    return m;
}

String RayTracer::computeMD5(const std::vector<Vec3f>& vertices) {
    unsigned char digest[16];
    MD5Buffer((void*)&vertices[0], sizeof(Vec3f)*vertices.size(), (unsigned int*)digest);
    // turn into string
    char ad[33];
    for ( int i = 0; i < 16; ++i )
        ::sprintf(ad+i*2, "%02x", digest[i] );
    ad[32] = 0;
    return FW::String( ad );
}

void RayTracer::loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles) {
	printf("Loading hierarchy from %s\n", filename);
    m_triangles = &triangles;
    m_bvh.m_triangles = &triangles;
    U32 hierarchy_size, tri_size;
    std::ifstream i_stream(filename, std::ifstream::binary);

    // Read the hierarchy
    i_stream.read(reinterpret_cast<char*>(&hierarchy_size), sizeof(hierarchy_size));
    std::vector<Node> tmp_hierarchy(hierarchy_size);
    i_stream.read(reinterpret_cast<char*>(tmp_hierarchy.data()), hierarchy_size * sizeof(Node));

    // Read the sorted triangle indices
    i_stream.read(reinterpret_cast<char*>(&tri_size), sizeof(tri_size));
    std::vector<U32> tmp_tri_indices(tri_size);
    i_stream.read(reinterpret_cast<char*>(tmp_tri_indices.data()), tri_size * sizeof(U32));
    FW_ASSERT(tri_size == triangles.size());

    // Copy to bvh
    m_bvh.m_hierarchy = tmp_hierarchy;
    m_bvh.m_tri_indices = tmp_tri_indices;
    // Sort the triangles so that the traversal works properly
    // We assume that the triangles are in the same order now as when the hierarchy was built
    m_bvh.sortTriangles();
#ifdef ISPC
    // Process the hierarchy for ispc
    m_bvh.rebranchForISPC(log2f(TRIS_IN_LEAF));
#endif // ISPC
}

void RayTracer::saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles) {
	printf("Saving hierarchy to %s\n", filename);
    U32 hierarchy_size = m_bvh.m_hierarchy.size();
    U32 tri_size = m_bvh.m_tri_indices.size();
    std::ofstream o_stream(filename, std::ofstream::binary);
    // Write hierarchy
    o_stream.write(reinterpret_cast<char*>(&hierarchy_size), sizeof(hierarchy_size));
    o_stream.write(reinterpret_cast<char*>(m_bvh.m_hierarchy.data()), hierarchy_size * sizeof(Node));
    // Write the sorted triangle indices 
    o_stream.write(reinterpret_cast<char*>(&tri_size), sizeof(tri_size));
    o_stream.write(reinterpret_cast<char*>(m_bvh.m_tri_indices.data()), tri_size * sizeof(U32));
}

void RayTracer::constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode split_mode) {
    m_triangles = &triangles;
    // Select the builder based on split_mode
	switch (split_mode) {
		case SplitMode_Sah:
			printf("SAH builder\n");
			m_bvh.constructSAH(triangles);
			printf("Size: %d\n", m_bvh.m_hierarchy.size());
			break;
		case SplitMode_SpatialMedian:
			printf("Spatial median builder\n");
			m_bvh.constructSpatialMedian(triangles);
			printf("Size: %d\n", m_bvh.m_hierarchy.size());
			break;
		case SplitMode_ObjectMedian:
			printf("Object median builder\n");
			m_bvh.constructObjectMedian(triangles);
			printf("Size: %d\n", m_bvh.m_hierarchy.size());
			break;
		case SplitMode_None:
			printf("No builder\n");
			m_bvh.construct(triangles);
			printf("Size: %d\n", m_bvh.m_hierarchy.size());
			break;
		default:
			printf("Default builder (SAH)\n");
			m_bvh.constructSAH(triangles);
			printf("Size: %d\n", m_bvh.m_hierarchy.size());
			break;
	} 
#ifdef ISPC
    // Process the hierarchy for ispc use
    // Intermediate nodes will have same number of leafs as leaf nodes have triangles 
    // (this should match the available vector lanes on the used processor)
	m_bvh.rebranchForISPC(log2f(TRIS_IN_LEAF));
#endif
}


RaycastResult RayTracer::raycast(const Vec3f& orig, const Vec3f& dir,std::vector<CheckNode>& b_nodes) const {
	++m_rayCount;

    F32 t_min = 1.0f, u_min = 0.0f, v_min = 0.0f;
    S32 i_min = -1;

    // Pre-calculate the inverse direction for performance
	Vec3f inv_dir;
	inv_dir.x = 1 / dir.x;
	inv_dir.y = 1 / dir.y;
	inv_dir.z = 1 / dir.z;

    // Add the root to the traversal stack
	b_nodes.emplace_back(&(Node&)m_bvh.m_hierarchy[0], 0.0f);	// Have to force the return value otherwise this returns const&
	while (!b_nodes.empty()) {
        CheckNode c_node = b_nodes.back();
		b_nodes.pop_back();
        // Ignore the node if we have allready found closer hit
		if (c_node.t >= t_min) {
			continue;
		}
		const Node& node = *c_node.node;
		if (node.left_child == -1) {
			// Naive loop over all triangles for leaf nodes
			for (U32 i = node.start; i < node.end; ++i) {
				F32 t, u, v;
				if ((*m_triangles)[i].intersect_woop(orig, dir, t, u, v)) {
					if ( t > 0.0f && t < t_min ) {
						i_min = i;
						t_min = t;
						u_min = u;
						v_min = v;
					}
				}
			}
		}
		else {
			F32 t_l = m_bvh.m_hierarchy[node.left_child].intersect(orig, dir, inv_dir);
			F32 t_r = m_bvh.m_hierarchy[node.right_child].intersect(orig, dir, inv_dir);
            // Add the intersected child nodes to the stack. Closer hit is added on top.
			if (t_l < t_min && t_r < t_min) {
				if (t_l < t_r) {
					b_nodes.emplace_back(&(Node&)m_bvh.m_hierarchy[node.right_child], t_r);
					b_nodes.emplace_back(&(Node&)m_bvh.m_hierarchy[node.left_child], t_l);
				} else {
					b_nodes.emplace_back(&(Node&)m_bvh.m_hierarchy[node.left_child], t_l);
					b_nodes.emplace_back(&(Node&)m_bvh.m_hierarchy[node.right_child], t_r);
				}
			} else if (t_l < t_min) {
				b_nodes.emplace_back(&(Node&)m_bvh.m_hierarchy[node.left_child], t_l);
			} else if (t_r < t_min) {
				b_nodes.emplace_back(&(Node&)m_bvh.m_hierarchy[node.right_child], t_r);
			}
		}
	}
	if (i_min != -1) {
		return RaycastResult(&(*m_triangles)[i_min], t_min, u_min, v_min, orig + t_min*dir, orig, dir);
    } else {
        return RaycastResult();
    }
}

#ifdef ISPC
RaycastResult RayTracer::ispcRaycast(const Vec3f & orig, const Vec3f & dir, std::vector<ISPCCheckNode>& b_nodes) const {
	++m_rayCount;

    F32 t_min = 1.0f, u_min = 0.0f, v_min = 0.0f;
    S32 i_min = -1;

    // Calculate the inverse direction
	Vec3f inv_dir;
	inv_dir.x = 1 / dir.x;
	inv_dir.y = 1 / dir.y;
	inv_dir.z = 1 / dir.z;
    // Convert to vectors to arrays for ispc
	float ispc_orig[3], ispc_dir[3], ispc_inv_dir[3];
	convertToISPC(orig, ispc_orig);
	convertToISPC(dir, ispc_dir);
	convertToISPC(inv_dir, ispc_inv_dir);

    // Add the root to the stack
	b_nodes.emplace_back(&(ISPCNode&)m_bvh.m_ispc_hierarchy[0], 0.0f);  // Have to force the return value otherwise this returns a const&
	while (!b_nodes.empty()) {
        ISPCCheckNode c_node = b_nodes.back();
		b_nodes.pop_back();
        // Ignore the node if we have allready found closer hit
		if (c_node.t >= t_min) {
			continue;
		}
		const ISPCNode& node = *c_node.node;
		if (node.n_children == 0) {
            // Intersect all the triangles at once using ispc
			ispc::TriangleIntersection intersections[TRIS_IN_LEAF];
			ispc::intersectTriangle(ispc_orig, ispc_dir, &m_bvh.m_ispc_triangles[node.first_triangle], node.n_triangles, intersections);
            // Check all the potential intersections
			for (U32 i = 0; i < node.n_triangles; i++) {
				ispc::TriangleIntersection intersection = intersections[i];
				if (intersection.hit && intersection.t < t_min) {
					t_min = intersection.t;
					u_min = intersection.u;
					v_min = intersection.v;
					i_min = node.first_triangle + i;
				}
			}
		}
		else {
            // Intersect all the bounding boxes using ispc
			float ts[TRIS_IN_LEAF];
			ispc::intersectAABB(ispc_orig, ispc_dir, ispc_inv_dir, &m_bvh.m_ispc_bbs[node.first_bb], node.n_children, ts);
            // Check all the potential hits
            U32 hit_nodes = 0;
			for (U32 i = 0; i < node.n_children; i++) {
				if (ts[i] < 1.0f) {
                    hit_nodes++;
					b_nodes.emplace_back(&(ISPCNode&)m_bvh.m_ispc_hierarchy[node.first_bb + i], ts[i]);
				}
			}
			auto sort_f = [](ISPCCheckNode& n1, ISPCCheckNode& n2) {
				return n1.t > n2.t;
			};
            // Sort the hits so the closest hits are visited first
			std::sort(b_nodes.end() - hit_nodes, b_nodes.end(), sort_f);
		}
	}
	if (i_min != -1) {
		return RaycastResult(&(*m_triangles)[i_min], t_min, u_min, v_min, orig + t_min*dir, orig, dir);
    } else {
        return RaycastResult();
    }
}
#endif // ISPC

} // namespace BDPT
} // namespace FW