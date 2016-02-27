#pragma once
#ifdef ISPC

#include "base/defs.hpp"

#include "intersect_ispc.h"
#include "RTTriangle.hpp"

namespace FW
{ namespace BDPT
{

struct ISPCNode {
	U32 n_children;
	U32 first_child;    // index of the first child
	U32 first_bb;	    // index of the first bounding box
	U32 n_triangles;
	U32 first_triangle;	// index of the first triangle
};

inline void convertToISPC(Vec3f original, float converted[3]) {
	for (U32 dim = 0; dim < 3; dim++) {
		converted[dim] = original[dim];
	}
}

inline void convertToISPC(Mat3f original, float converted[3][3])     {
	F32* mat_ptr = original.getPtr();
	for (U32 i = 0; i < 3; i++) {
		for (U32 j = 0; j < 3; j++) {
			converted[i][j] = mat_ptr[3*i + j];
		}
	}
}

inline ispc::Triangle convertToISPC(const RTTriangle& original) {
	ispc::Triangle converted;
	convertToISPC(original.m_data.M, converted.M);
	convertToISPC(original.m_data.N, converted.N);
	return converted;
}

inline ispc::AABB convertToISPC(const AABB& original) {
	ispc::AABB converted;
	convertToISPC(original.max, converted.max);
	convertToISPC(original.min, converted.min);
	return converted;
}

} //namespace BDPT
} //namespace FW
#endif // ISPC