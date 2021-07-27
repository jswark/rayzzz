#ifndef BVH_H
#define BVH_H

#include "hittable.h"
#include "hittablelist.h"
#include "rtweekend.h"
#include <algorithm>
#include <vector>

struct BVHNode {
  vec3 m_minBounds{};
  vec3 m_instanceIndex{};
  vec3 m_maxBounds{};
  std::vector<point3> coord2{}; // todo add to aabbs
  int m_nodeOffset = -1;
};
std::vector<BVHNode> bvh(100);

class bvh_node : public hittable {
public:
  bvh_node(const hittable_list &list, double time0, double time1, int index)
      : // bvh.resize(2 * list.objects.size() - 1);
        bvh_node(list.objects, 0, list.objects.size(), time0, time1, &index){
            setDepthFirstVisitOrder();
        };

  bvh_node(const std::vector<shared_ptr<hittable>> &src_objects, size_t start,
           size_t end, double time0, double time1, int *index);

  virtual bool hit(const ray &r, double t_min, double t_max,
                   hit_record &rec) const override;

  virtual bool bounding_box(double time0, double time1,
                            aabb &output_box) const override;

  virtual std::vector<point3> getCoord() const override{};

  bool hitAny(const ray &r);

  void setDepthFirstVisitOrder(int nodeId, int nextId, int& order);
  void setDepthFirstVisitOrder();

  void set_left(int n, std::vector<point3> coord);
  void set_right(int n, std::vector<point3> coord);
  void set_box(int n, vec3 maxB, vec3 minB, std::vector<point3> coord);

public:
  shared_ptr<hittable> left;
  shared_ptr<hittable> right;
  aabb box;
};

bool bvh_node::bounding_box(double time0, double time1,
                            aabb &output_box) const {
  output_box = box;
  return true;
}

bool bvh_node::hit(const ray &r, double t_min, double t_max,
                   hit_record &rec) const {
  // check if box hitted
  if (!box.hit(r, t_min, t_max))
    return false;

  // check if triangle hitted
  bool hit_left = left->hit(r, t_min, t_max, rec);
  bool hit_right = right->hit(r, t_min, hit_left ? rec.t : t_max, rec);

  return hit_left || hit_right;
}

bool intersectBox(const ray &r, vec3 pmin, vec3 pmax) {
  double t_min = 0.001;
  double t_max = infinity;

  for (int a = 0; a < 3; a++) {
    auto t0 = fmin((pmin[a] - r.origin()[a]) / r.direction()[a],
                   (pmax[a] - r.origin()[a]) / r.direction()[a]);
    auto t1 = fmax((pmin[a] - r.origin()[a]) / r.direction()[a],
                   (pmax[a] - r.origin()[a]) / r.direction()[a]);

    t_min = fmax(t0, t_min);
    t_max = fmin(t1, t_max);

    if (t_max <= t_min)
      return false;
  }
  return true;
}

bool intersectRayTri(const ray &r, vec3 v0, vec3 v1, vec3 v2) {
  float t = 0.f, u = 0.f, v = 0.f;

  vec3 v0v1 = v1 - v0;
  vec3 v0v2 = v2 - v0;

  vec3 pvec = cross(r.direction(), v0v2);

  float det = dot(pvec, v0v1);
  float kEpsilon = 0.00001;

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

  return true;
}

bool bvh_node::hitAny(const ray &r) {
  int nodeIndex = 0;

  while (nodeIndex != -1) {
    BVHNode node;
    node.m_minBounds = bvh[nodeIndex * 2 + 0].m_minBounds;
    node.m_minBounds = bvh[nodeIndex * 2 + 1].m_maxBounds;

    std::vector<vec3> primitiveIndex =
        bvh[nodeIndex * 2 + 0].coord2; // actually the coords of the triangle

    if (!(primitiveIndex[0] ==
          vec3{0, 0, 0}) /* coord exist ?? */) { // leaf node

      if (intersectRayTri(r, primitiveIndex[0], primitiveIndex[1],
                          primitiveIndex[2])) {
        return true;
      }
    }

    else if (!intersectBox(r, node.m_minBounds, node.m_maxBounds)) {
      ++nodeIndex;
      continue;
    }

    nodeIndex = bvh[nodeIndex * 2 + 1].m_nodeOffset; // next
  }
  return false;
}

inline bool box_compare(const shared_ptr<hittable> a,
                        const shared_ptr<hittable> b, int axis) {
  aabb box_a;
  aabb box_b;

  if (!a->bounding_box(0, 0, box_a) || !b->bounding_box(0, 0, box_b))
    std::cerr << "No bounding box in bvh_node constructor.\n";

  return box_a.min().e[axis] < box_b.min().e[axis];
}

bool box_x_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
  return box_compare(a, b, 0);
}

bool box_y_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
  return box_compare(a, b, 1);
}

bool box_z_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
  return box_compare(a, b, 2);
}

void bvh_node::setDepthFirstVisitOrder(int nodeId, int nextId, int& order)
{
  BVHNode& node = bvh[nodeId];
  order++;
  node.m_nodeOffset = nextId;

  if (bvh[nodeId*2].m_instanceIndex.x() != -1 /* coord valid */) // check left
  {
    setDepthFirstVisitOrder(nodeId*2, nodeId*2 + 1, order);
  }

  if (bvh[nodeId*2 + 1].m_instanceIndex.x() != -1)
  {
    setDepthFirstVisitOrder(nodeId*2 + 1, nextId, order);
  }
}

void bvh_node::setDepthFirstVisitOrder()
{
  int order = 0;
  setDepthFirstVisitOrder(0, -1, order);
}

void bvh_node::set_left(int n, std::vector<point3> coord) {
  bvh[n].m_instanceIndex = coord[0];
} // set x

void bvh_node::set_right(int n, std::vector<point3> coord) {
  bvh[n].m_instanceIndex = coord[0];
} // set x

void bvh_node::set_box(int n, vec3 maxB, vec3 minB, std::vector<point3> coord) {
  bvh[n].m_maxBounds = maxB;
  bvh[n].m_minBounds = minB;
  bvh[n].coord2 = coord; // set yz
}

bvh_node::bvh_node(const std::vector<shared_ptr<hittable>> &src_objects,
                   size_t start, size_t end, double time0, double time1,
                   int *index) {
  int currindex = (*index);
  auto objects =
      src_objects; // Create a modifiable array of the source scene objects
  int axis = random_int(0, 2);
  auto comparator = (axis == 0)   ? box_x_compare
                    : (axis == 1) ? box_y_compare
                                  : box_z_compare;

  size_t object_span = end - start;

  if (object_span == 1) {
    left = right = objects[start];

    set_right(2 * (*index) + 1, objects[start]->getCoord());
    set_left(2 * (*index), objects[start]->getCoord());
    (*index) += 1;
  } else if (object_span == 2) {
    if (comparator(objects[start], objects[start + 1])) {
      left = objects[start];
      right = objects[start + 1];

      set_right(2 * (*index) + 1, objects[start + 1].get()->getCoord());
      set_left(2 * (*index), objects[start]->getCoord());
      (*index) += 2;
    } else {
      left = objects[start + 1];
      right = objects[start];

      set_right(2 * (*index) + 1, objects[start]->getCoord());
      set_left(2 * (*index), objects[start + 1]->getCoord());
      (*index) += 2;
    }
  } else {
    std::sort(objects.begin() + start, objects.begin() + end, comparator);

    auto mid = start + object_span / 2;
    left = make_shared<bvh_node>(objects, start, mid, time0, time1, index);
    right = make_shared<bvh_node>(objects, mid, end, time0, time1, index);
  }

  aabb box_left, box_right;

  if (!left->bounding_box(time0, time1, box_left) ||
      !right->bounding_box(time0, time1, box_right))
    std::cerr << "No bounding box in bvh_node constructor.\n";

  box = surrounding_box(box_left, box_right);

  set_box(2 * (currindex), box.max(), box.min(),
          objects[start + 1]->getCoord());
  set_box(2 * (currindex) + 1, box.max(), box.min(),
          objects[start + 1]->getCoord());
}

#endif