#ifndef BVH_H
#define BVH_H

#include "hittable.h"
#include "hittablelist.h"
#include "rtweekend.h"
#include <algorithm>
#include <vector>

struct BVHNode {
  vec3 m_minBounds;
  int m_instanceIndex = -1;
  vec3 m_maxBounds;
  int m_nodeOffset;
};
std::vector<BVHNode> bvh(100);

class bvh_node : public hittable {
public:
  bvh_node(const hittable_list &list, double time0, double time1, int index)
      : // bvh.resize(2 * list.objects.size() - 1);
        bvh_node(list.objects, 0, list.objects.size(), time0, time1, &index){};

  bvh_node(const std::vector<shared_ptr<hittable>> &src_objects, size_t start,
           size_t end, double time0, double time1, int *index);

  virtual bool hit(const ray &r, double t_min, double t_max,
                   hit_record &rec) const override;

  virtual bool bounding_box(double time0, double time1,
                            aabb &output_box) const override;

  virtual int getIndex() const override{ return 0;};

  void set_left(int n, int index);
  void set_right(int n, int index);
  void set_box(int n, vec3 maxB, vec3 minB);

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

void bvh_node::set_left(int n, int index) { bvh[n].m_instanceIndex = index; }

void bvh_node::set_right(int n, int index) { bvh[n].m_instanceIndex = index; }

void bvh_node::set_box(int n, vec3 maxB, vec3 minB) {
  bvh[n].m_maxBounds = maxB;
  bvh[n].m_minBounds = minB;
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

    set_right(2 * (*index) + 1, objects[start]->getIndex());
    set_left(2 * (*index), objects[start]->getIndex());
    (*index) += 1;
  } else if (object_span == 2) {
    if (comparator(objects[start], objects[start + 1])) {
      left = objects[start];
      right = objects[start + 1];

      set_right(2 * (*index) + 1, objects[start + 1].get()->getIndex());
      set_left(2 * (*index), objects[start]->getIndex());
      (*index) += 2;
    } else {
      left = objects[start + 1];
      right = objects[start];

      set_right(2 * (*index) + 1, objects[start]->getIndex());
      set_left(2 * (*index), objects[start + 1]->getIndex());
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

  set_box(2 * (currindex), box.max(), box.min());
  set_box(2 * (currindex) + 1, box.max(), box.min());
}

#endif