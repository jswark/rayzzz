#pragma once

#include "aabb.h"
#include "hittable.h"
#include "ray.h"
#include "rtweekend.h"
#include "vec3.h"

#include <cmath>
#include <cstdlib>
#include <utility>

class triangle : public hittable
{
public:
    triangle()
    {
    }
    triangle(point3 _v0, point3 _v1, point3 _v2, shared_ptr<material> m, int index)
        : v0(_v0), v1(_v1), v2(_v2), material(m), index(index)
    {
    }

    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override
    {
        float minX = min(v0.x(), min(v1.x(), v2.x()));
        float minY = min(v0.y(), min(v1.y(), v2.y()));
        float minZ = min(v0.z(), min(v1.z(), v2.z()));

        float maxX = max(v0.x(), max(v1.x(), v2.x()));
        float maxY = max(v0.y(), max(v1.y(), v2.y()));
        float maxZ = max(v0.z(), max(v1.z(), v2.z()));

        const float eps = 1e-7f;
        // need to pad aabb to prevent from ultra thin box (zero width)
        output_box = aabb(point3(minX, minY, minZ), point3(maxX + eps, maxY + eps, maxZ + eps));
        
        return true;
    }

    virtual int getIndex() const override
    {
        return index;
    };

    virtual std::vector<point3> getCoord() const override
    {
        return { v0, v1, v2 };
    };

    virtual void reconstruct(int index) const override{};

    point3 v0, v1, v2;
    int index = -1;
    shared_ptr<material> material;
};

bool triangle::hit(const ray& r, double t_min, double t_max, hit_record& rec) const
{

    vec3 normal = cross(v1 - v0, v2 - v0);

    float t = 0.f, u = 0.f, v = 0.f;

    vec3 v0v1 = v1 - v0;
    vec3 v0v2 = v2 - v0;

    vec3 pvec = cross(r.direction(), v0v2);

    float det = dot(pvec, v0v1);
    float kEpsilon = 0.00001;

    // if the determinant is negative the triangle is backfacing
    // if the determinant is close to 0, the ray misses the triangle
    if (det < kEpsilon)
        return false;

    float invDet = 1 / det;

    vec3 tvec = r.origin() - v0;
    u = dot(tvec, pvec) * invDet;

    if (u < 0 || u > 1)
        return false;

    vec3 qvec = cross(tvec, v0v1);
    v = dot(r.direction(), qvec) * invDet;
    if (v < 0 || u + v > 1)
        return false;

    t = dot(v0v2, qvec) * invDet;

    if (t < 0)
        return false;

    rec.p = r.at(t);
    rec.t = t;
    rec.set_face_normal(r, normal);
    rec.mat_ptr = material;

    return true;
}
