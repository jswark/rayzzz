#ifndef TRIANGLE_
#define TRIANGLE_

#include "hittable.h"
#include "vec3.h"
#include <cstdlib>
#include <utility>

#include "aabb.h"
#include "hittable.h"
#include "rtweekend.h"

template <class T> const T &min(const T &a, const T &b) {
  return (b < a) ? b : a;
}

template <class T> const T &max(const T &a, const T &b) {
  return (b > a) ? b : a;
}

class triangle : public hittable {
public:
  triangle() {}
  triangle(point3 _v0, point3 _v1, point3 _v2, shared_ptr<material> m, int index)
      : v0(_v0), v1(_v1), v2(_v2), material(m) , index(index){}

  virtual bool hit(const ray &r, double t_min, double t_max,
                   hit_record &rec) const override;

  virtual bool bounding_box(double time0, double time1,
                            aabb &output_box) const override {
    output_box = aabb(min(v0, min(v1, v2)), max(v0, max(v1, v2)));
    return true;
  }

  virtual int getIndex() const override{
      return index;
  };

  point3 v0, v1, v2;
  int index;
  shared_ptr<material> material;
};

bool triangle::hit(const ray &r, double t_min, double t_max,
                   hit_record &rec) const {

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

#endif
