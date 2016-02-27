#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include "rtutil.hpp"

namespace FW
{

struct Node {
	AABB bb;
	U32 start, end;		//[start, end)
	S32 left_child;
	S32 right_child;

	F32 intersect(const Vec3f& orig, const Vec3f& dir, const Vec3f& inv_dir) const;
};

} //namespace FW
