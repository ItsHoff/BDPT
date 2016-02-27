// Structs for communicating the intersected primimites
struct AABB {
    float min[3], max[3];
};

struct Triangle {
    float M[3][3];
    float N[3];
};

// Struct containing the triangle intersection results
struct TriangleIntersection {
    float t, u, v;
    bool hit;
};


export void intersectAABB(uniform const float orig[], uniform const float dir[], uniform const float inv_dir[], 
                          uniform const AABB bbs[], uniform int count, uniform float ts[]) {
    foreach(i = 0 ... count) {
        float t1, t2;
        float t_start = 0.0;
        float t_end = 1.0;
        AABB bb = bbs[i];
        for (uniform int dim = 0; dim < 3; dim++) {
            if (dir[dim] == 0) {
                if (orig[dim] < bb.min[dim] || orig[dim] > bb.max[dim]) {
                    t_start = 1.0;  // We can't hit the box but no use stopping one instance
                }
            } else if (dir[dim] > 0) {
                t1 = (bb.min[dim] - orig[dim]) * inv_dir[dim];
                t2 = (bb.max[dim] - orig[dim]) * inv_dir[dim];
            } else {
                t1 = (bb.max[dim] - orig[dim]) * inv_dir[dim];
                t2 = (bb.min[dim] - orig[dim]) * inv_dir[dim];
            }
            t_start = max(t1, t_start);
            t_end = min(t2, t_end);
        }
        if (t_end < 0.0f || t_start > t_end) {
            ts[i] = 1.0;    // No hit
        } else if (t_start > 0.0) {
            ts[i] = t_start;
        } else {
            ts[i] = 0.0;    // We're inside the box
        }
    }
}

// Multiply column-major 3x3 matrix with a vector
float<3> multiply(float mat[3][3], uniform const float vec[3]) {
    float<3> result = {0, 0, 0};
    for (uniform int col = 0; col < 3; col++) {
        for (uniform int row = 0; row < 3; row++) {
            result[row] += mat[col][row]*vec[col];
        }
    }
    return result;
}

// Woop intersect the triangles
export void intersectTriangle(uniform const float orig[], uniform const float dir[], uniform const Triangle triangles[], uniform int count,
                              uniform TriangleIntersection intersections[]) {
    foreach(i = 0 ... count) {
        Triangle tri = triangles[i];
        float<3> n_vec = {tri.N[0], tri.N[1], tri.N[2]};
        float<3> transformed_orig = multiply(tri.M, orig) + n_vec;
        float<3> transformed_dir = multiply(tri.M, dir);
        
        TriangleIntersection* intersection = &intersections[i];
        intersection->t = -transformed_orig.z / transformed_dir.z;
        intersection->u = transformed_orig.x + transformed_dir.x * intersection->t;
        intersection->v = transformed_orig.y + transformed_dir.y * intersection->t;
        // Check whether we actually hit the triangle
        if (!isnan(intersection->t) && intersection->u > 0.0 && intersection->v > 0.0 && intersection->u + intersection->v < 1.0 && intersection->t > 0.0) {
            intersection->hit = true;
        } else {
            intersection->hit = false;
        }
    }
}
