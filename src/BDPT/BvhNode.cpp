#include "BvhNode.hpp"

namespace FW
{ namespace BDPT
{

F32 Node::intersect(const Vec3f& orig, const Vec3f& dir, const Vec3f& inv_dir) const {
	Vec3f t1 = (bb.min - orig) * inv_dir;
	Vec3f t2 = (bb.max - orig) * inv_dir;
	for (U32 dim = 0; dim < 3; dim++) {
		if (dir[dim] == 0) {
			if (orig[dim] < bb.min[dim] || orig[dim] > bb.max[dim]) {
                // Can't hit the node
				return FLT_MAX;
			} else {
                // Protection from division by zero
				t1[dim] = -FLT_MAX;
				t2[dim] = FLT_MAX;
			}
		} else if (dir[dim] < 0) {
            // t1 must be the smaller value
			std::swap(t1[dim], t2[dim]);
		}
	}
	F32 t_start = t1.max();
	F32 t_end = t2.min();
	if (t_end < 0.0f || t_start > t_end) {
        // No hit
		return FLT_MAX;
	} else if (t_start > 0.0f) {
		return t_start;
	} else {	
        // We're inside the box
		return 0.0f;
	}
}

} //namespace BDPT
} //namespace FW