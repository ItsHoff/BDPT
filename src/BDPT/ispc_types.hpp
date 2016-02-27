#pragma once
#ifdef ISPC

#include "intersect_ispc.h"
#include "base/defs.hpp"
#include "RTTriangle.hpp"
#include "rtutil.hpp"

namespace FW
{

struct ISPCNode {
	U32 n_children;
	U32 first_child;  // index of the first child
	U32 first_bb;	 // index of the first bounding box
	U32 n_triangles;
	U32 first_triangle;	// index of the first triangle
};

void convertToISPC(Vec3f original, float converted[3]);
void convertToISPC(Mat3f original, float converted[3][3]);
ispc::Triangle convertToISPC(const RTTriangle& original);
ispc::AABB convertToISPC(const AABB& original);

} //namespace FW
#endif // ISPC