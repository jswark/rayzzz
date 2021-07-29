#pragma once

#include "hittable.h"
#include "hittablelist.h"
#include "material.h"
#include "rtweekend.h"

#include <algorithm>
#include <utility>
#include <vector>

struct BVHNode
{
    vec3 m_minBounds{};
    int m_instanceIndex = -1;
    vec3 m_maxBounds{};
    std::vector<point3> coord{}; // todo add to aabbs
    int m_nodeOffset = -1;
    bool isLeaf = false;
};

std::vector<BVHNode> bvh(100);

class bvh_node : public hittable
{
public:
    bvh_node(const hittable_list& list, double time0, double time1, int index)
        : bvh_node(list.objects, 0, list.objects.size(), time0, time1, index)
    {
        bvh.resize(pow(2, (list.objects.size() - 1)));
        reconstruct(0);
    };

    bvh_node(const std::vector<shared_ptr<hittable>>& src_objects,
             size_t start,
             size_t end,
             double time0,
             double time1,
             int index);

    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;

    virtual std::vector<point3> getCoord() const override
    {
        return {};
    };
    virtual int getIndex() const override
    {
        return 0;
    };

    virtual void reconstruct(int index) const override;

    bool hitAny(const ray& r, hit_record& rec);

public:
    shared_ptr<hittable> left;
    shared_ptr<hittable> right;
    aabb box;
    int indexLeft = -1;
    int indexRight = -1;
    std::vector<point3> coordLeft{};
    std::vector<point3> coordRight{};
};

void bvh_node::reconstruct(int index) const
{
    if (index == 0) // root
    {
        bvh[0].m_instanceIndex = indexLeft;
        bvh[0].m_maxBounds = box.max();
        bvh[0].m_minBounds = box.min();
        bvh[0].coord = coordLeft;
    }

    if (indexLeft != -1)
    {
        bvh[2 * index + 1].isLeaf = true;
    }
    bvh[2 * index + 1].m_instanceIndex = indexLeft;
    bvh[2 * index + 1].m_maxBounds = box.max();
    bvh[2 * index + 1].m_minBounds = box.min();
    bvh[2 * index + 1].coord = coordLeft;
    if (2 * index + 2 < bvh.size())
        bvh[2 * index + 1].m_nodeOffset = 2 * index + 2; // set right bro

    if (indexRight != -1)
    {
        bvh[2 * index + 2].isLeaf = true;
    }
    bvh[2 * index + 2].m_instanceIndex = indexRight;
    bvh[2 * index + 2].m_maxBounds = box.max();
    bvh[2 * index + 2].m_minBounds = box.min();
    bvh[2 * index + 2].coord = coordRight;


    left->reconstruct(2 * index + 1);
    right->reconstruct(2 * index + 2);
}

bool bvh_node::bounding_box(double time0, double time1, aabb& output_box) const
{
    output_box = box;
    return true;
}

bool bvh_node::hit(const ray& r, double t_min, double t_max, hit_record& rec) const
{
    // check if box hitted
    if (!box.hit(r, t_min, t_max))
        return false;

    // check if triangle hitted
    bool hit_left = left->hit(r, t_min, t_max, rec);
    bool hit_right = right->hit(r, t_min, hit_left ? rec.t : t_max, rec);

    return hit_left || hit_right;
}

bool intersectBox(const ray& r, vec3 pmin, vec3 pmax)
{
    double t_min = 0.001;
    double t_max = infinity;

    for (int a = 0; a < 3; a++)
    {
        auto t0 = fmin((pmin[a] - r.origin()[a]) / r.direction()[a], (pmax[a] - r.origin()[a]) / r.direction()[a]);
        auto t1 = fmax((pmin[a] - r.origin()[a]) / r.direction()[a], (pmax[a] - r.origin()[a]) / r.direction()[a]);

        t_min = fmax(t0, t_min);
        t_max = fmin(t1, t_max);

        if (t_max <= t_min)
            return false;
    }
    return true;
}

bool intersectRayTri(const ray& r, vec3 v0, vec3 v1, vec3 v2, hit_record& rec)
{
    float t = 0.f, u = 0.f, v = 0.f;

    vec3 v0v1 = v1 - v0;
    vec3 v0v2 = v2 - v0;

    vec3 normal = cross(v1 - v0, v2 - v0);

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

    rec.p = r.at(t);
    rec.t = t;
    rec.set_face_normal(r, normal);
    rec.mat_ptr = make_shared<lambertian>(color(.65, .05, .05));

    return true;
}

bool hitAny(const ray& r, hit_record& rec)
{
    int nodeIndex = 1;

    while (nodeIndex != -1)
    {
        BVHNode& node = bvh[nodeIndex];
        if (node.isLeaf)
        { // leaf node
            if (intersectRayTri(r, node.coord[0], node.coord[1], node.coord[2], rec))
            {
                return true;
            }
        }
        else if (!intersectBox(r, node.m_minBounds, node.m_maxBounds))
        {
            nodeIndex = node.m_nodeOffset;
            continue;
        }
        std::cout << nodeIndex << " " << node.m_nodeOffset << std::endl;
        if (node.m_nodeOffset * 2 + 2 == nodeIndex)
        { // very right last node
            nodeIndex = -1;
            continue;
        }

        if (node.isLeaf && node.m_nodeOffset != -1)
            nodeIndex = node.m_nodeOffset;
        else if (!node.isLeaf && 2 * nodeIndex + 1 < bvh.size())
            nodeIndex = 2 * nodeIndex + 1;
        else
            nodeIndex = node.m_nodeOffset;
    }
    return false;
}

inline bool box_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b, int axis)
{
    aabb box_a;
    aabb box_b;

    if (!a->bounding_box(0, 0, box_a) || !b->bounding_box(0, 0, box_b))
        std::cerr << "No bounding box in bvh_node constructor.\n";

    return box_a.min().e[axis] < box_b.min().e[axis];
}

bool box_x_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b)
{
    return box_compare(a, b, 0);
}

bool box_y_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b)
{
    return box_compare(a, b, 1);
}

bool box_z_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b)
{
    return box_compare(a, b, 2);
}

bvh_node::bvh_node(
    const std::vector<shared_ptr<hittable>>& src_objects, size_t start, size_t end, double time0, double time1, int index)
{
    auto objects = src_objects; // Create a modifiable array of the source scene objects
    int axis = random_int(0, 2);
    auto comparator = (axis == 0) ? box_x_compare : (axis == 1) ? box_y_compare : box_z_compare;

    size_t object_span = end - start;

    if (object_span == 1)
    {
        left = right = objects[start];
        indexLeft = objects[start]->getIndex();
        indexRight = -1;

        coordLeft = objects[start]->getCoord();
    }
    else if (object_span == 2)
    {
        if (comparator(objects[start], objects[start + 1]))
        {
            left = objects[start];
            right = objects[start + 1];

            indexLeft = objects[start]->getIndex();
            indexRight = objects[start + 1]->getIndex();

            coordLeft = objects[start]->getCoord();
            coordRight = objects[start + 1]->getCoord();
        }
        else
        {
            left = objects[start + 1];
            right = objects[start];

            indexLeft = objects[start + 1]->getIndex();
            indexRight = objects[start]->getIndex();

            coordLeft = objects[start + 1]->getCoord();
            coordRight = objects[start]->getCoord();
        }
    }
    else
    {
        std::sort(objects.begin() + start, objects.begin() + end, comparator);
        auto mid = start + object_span / 2;

        left = make_shared<bvh_node>(objects, start, mid, time0, time1, index + 1);
        right = make_shared<bvh_node>(objects, mid, end, time0, time1, index + 2);
    }

    aabb box_left, box_right;

    if (!left->bounding_box(time0, time1, box_left) || !right->bounding_box(time0, time1, box_right))
        std::cerr << "No bounding box in bvh_node constructor.\n";

    box = surrounding_box(box_left, box_right);
}
