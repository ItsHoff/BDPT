#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include "base\Math.hpp"


namespace FW
{ namespace BDPT
{

struct AABB {
    Vec3f min, max;     // Corners of the bb

    inline AABB() : 
        min(), max() {
    }
    inline AABB(const Vec3f& min, const Vec3f& max) :
        min(min), max(max) {
    }

    // Calculate the surface area of the bb for SAH
    inline F32 area() const {
        Vec3f d(max - min);
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }
};


struct Node {
    AABB bb;
    U32 start, end;		// triangles contained in the node [start, end)
    S32 left_child;     // index of the left child in the hierarchy
    S32 right_child;    // index of the right child in the hierarchy

    // Find the t value of the ray-bb intersection
    // Returns t > 1.0 if no hit
    F32 intersect(const Vec3f& orig, const Vec3f& dir, const Vec3f& inv_dir) const;
};

} //namespace BDPT
} //namespace FW
