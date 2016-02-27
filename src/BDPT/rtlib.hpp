#pragma once


#include "RaycastResult.hpp"

#include "rtutil.hpp"
#include "base/Math.hpp"

#include <vector>
#include <memory>


namespace FW {
namespace rtlib {


// See formBasis in RayTracer.
Mat3f formBasis(const Vec3f& n);


// Private implementation of the ray tracer.
class RayTracer_;


// Library implementation of RayTracer.
class RayTracer
{
public:
                        RayTracer				(void);
                        ~RayTracer				(void);

    void				constructHierarchy		(std::vector<RTTriangle>& triangles, SplitMode splitMode);

    void				saveHierarchy			(const char* filename, const std::vector<RTTriangle>& triangles);
    void				loadHierarchy			(const char* filename, std::vector<RTTriangle>& triangles);

    RaycastResult		raycast					(const Vec3f& orig, const Vec3f& dir) const;

private:
    // Private implementation of the ray tracer.
    std::unique_ptr<RayTracer_> back_;
};


}
}


