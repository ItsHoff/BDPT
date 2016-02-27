#pragma once

/*
 * utilities for internal use etc. in the raytracer class and the bvh construction
 */

#include "base/Math.hpp"

#include <iostream>


namespace FW {


enum SplitMode {
	SplitMode_SpatialMedian,
	SplitMode_ObjectMedian,
	SplitMode_Sah,
	SplitMode_None,
	SplitMode_Linear
};

struct Plane : public Vec4f {
    inline float dot(const Vec3f& p) const {
        return p.x * x + p.y * y + p.z * z + w;
    }
};


struct AABB {
    Vec3f min, max;
    inline AABB() : min(), max() {}
    inline AABB(const Vec3f& min, const Vec3f& max) : min(min), max(max) {}
    inline F32 area() const {
        Vec3f d(max - min);
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }
};


inline std::ostream& operator<<(std::ostream& os, const FW::Vec3f& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

inline std::ostream& operator<<(std::ostream& os, const FW::Vec4f& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")";
}

inline std::ostream& operator<<(std::ostream& os, const AABB& bb) {
    return os << "BB(" << bb.min << ", " << bb.max << ")";
}


}
