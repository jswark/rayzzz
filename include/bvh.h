#pragma once

#include "hittable.h"
#include "hittablelist.h"
#include "material.h"
#include "rtweekend.h"

#include <algorithm>
#include <assert.h>
#include <utility>
#include <vector>

class BVHBuilder : hittable
{
private:
    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override
    {
        return 0;
    };

    virtual std::vector<point3> getCoord() const override
    {
        return {};
    };

    virtual int getIndex() const override
    {
        return 0;
    };

    virtual void reconstruct(int index) override{};
    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override
    {
        return 0;
    };

    void recreate(const hittable_list& list)
    {
        const uint32_t leafCount = getNextPow2(list.objects.size());
        const uint32_t nodesCount = 2 * leafCount; // full bin tree
        BVHBuilder::bvh.resize(nodesCount);
        inner.reconstruct(0);

        int rightSib = 1;
        setDepthFirstVisitOrder(0, -1, rightSib);
        // set last offset
        for (int i = BVHBuilder::bvh.size() - 1; i > 0; --i)
        {
            if (BVHBuilder::bvh[i].m_instanceIndex != -1)
            {
                BVHBuilder::bvh[i].m_nodeOffset = -1;
                break;
            }
        }
    };

    void setDepthFirstVisitOrder(int nodeId, int nextId, int& savedRight);

    unsigned int getNextPow2(unsigned int n) // compute the next highest power of 2 of 32-bit v
    {
        n--;
        n |= n >> 1;
        n |= n >> 2;
        n |= n >> 4;
        n |= n >> 8;
        n |= n >> 16;
        n++;

        return n;
    }

    class bvhNodeInternal : public hittable
    {
        shared_ptr<hittable> left;
        shared_ptr<hittable> right;
        aabb box;
        int indexLeft = -1;
        int indexRight = -1;
        std::vector<point3> coordLeft{};
        std::vector<point3> coordRight{};

        // 1) create bvh
        bvhNodeInternal(const hittable_list& list, double time0, double time1, BVHBuilder* outer)
            : bvhNodeInternal(list.objects, 0, list.objects.size(), time0, time1)
        {
            outer = outer;
        };

        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;

        virtual std::vector<point3> getCoord() const override
        {
            return {};
        };
        virtual int getIndex() const override
        {
            return 0;
        };


    public:
        bvhNodeInternal(
            const std::vector<shared_ptr<hittable>>& src_objects, size_t start, size_t end, double time0, double time1);
        virtual void reconstruct(int index) override;
        virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override;
    };

    bvhNodeInternal inner;

public:
    struct BVHNode
    {
        vec3 m_minBounds{};
        int m_instanceIndex = -1;
        vec3 m_maxBounds{};
        std::vector<point3> coord{}; // todo add to aabbs
        int m_nodeOffset = -1;
        bool isLeaf = false;
    };

    static std::vector<BVHNode> bvh; // can't live w/o static :((

    // 2) get array of nodes ??
    static std::vector<BVHNode> getBVH()
    {
        return BVHBuilder::bvh;
    }

    // create bvh
    BVHBuilder(const hittable_list& list, double time0, double time1)
        : inner(list.objects, 0, list.objects.size(), time0, time1)
    {
        recreate(list);
    };
};

void BVHBuilder::bvhNodeInternal::reconstruct(int index)
{
    int leftIndex = 2 * index + 1;
    int rightIndex = 2 * index + 2;

    if (index == 0) // root
    {
        BVHBuilder::bvh[0].m_instanceIndex = indexLeft;
        BVHBuilder::bvh[0].m_maxBounds = box.max();
        BVHBuilder::bvh[0].m_minBounds = box.min();
        BVHBuilder::bvh[0].coord = coordLeft;
    }

    if (indexLeft != -1)
    {
        BVHBuilder::bvh[leftIndex].isLeaf = true;
    }

    BVHBuilder::bvh[leftIndex].m_instanceIndex = indexLeft;
    BVHBuilder::bvh[leftIndex].m_maxBounds = box.max();
    BVHBuilder::bvh[leftIndex].m_minBounds = box.min();
    BVHBuilder::bvh[leftIndex].coord = coordLeft;

    if (rightIndex < BVHBuilder::bvh.size())
    {
        BVHBuilder::bvh[leftIndex].m_nodeOffset = rightIndex; // set right bro
    }

    if (indexRight != -1)
    {
        BVHBuilder::bvh[rightIndex].isLeaf = true;
    }

    BVHBuilder::bvh[rightIndex].m_instanceIndex = indexRight;
    BVHBuilder::bvh[rightIndex].m_maxBounds = box.max();
    BVHBuilder::bvh[rightIndex].m_minBounds = box.min();
    BVHBuilder::bvh[rightIndex].coord = coordRight;

    left->reconstruct(leftIndex);
    right->reconstruct(rightIndex);
}

void BVHBuilder::setDepthFirstVisitOrder(int nodeId, int nextId, int& savedRight)
{
    int leftIndex = 2 * nodeId + 1;
    int rightIndex = 2 * nodeId + 2;

    if (nodeId != 0) // root
    {
        BVHBuilder::bvh[nodeId].m_nodeOffset = nextId;
    }

    if (nodeId < BVHBuilder::bvh.size() && !BVHBuilder::bvh[nodeId].isLeaf && savedRight != -1)
    {
        BVHBuilder::bvh[savedRight].m_nodeOffset = nodeId;
        savedRight = -1;
    }

    if (leftIndex < BVHBuilder::bvh.size() && !BVHBuilder::bvh[leftIndex].isLeaf) // not leaf, check left
    {
        setDepthFirstVisitOrder(leftIndex, rightIndex, savedRight);
    }

    if (rightIndex < BVHBuilder::bvh.size() && !BVHBuilder::bvh[rightIndex].isLeaf) // not leaf, check right
    {
        setDepthFirstVisitOrder(rightIndex, nextId, savedRight);
    }

    if (rightIndex < BVHBuilder::bvh.size() && BVHBuilder::bvh[rightIndex].isLeaf) // save offset for right leaf
    {
        savedRight = rightIndex; // index need next offset
    }
}

bool BVHBuilder::bvhNodeInternal::bounding_box(double time0, double time1, aabb& output_box) const
{
    output_box = box;
    return true;
}

bool BVHBuilder::bvhNodeInternal::hit(const ray& r, double t_min, double t_max, hit_record& rec) const
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
    vec3 normal = cross(v1 - v0, v2 - v0);
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

    rec.p = r.at(t);
    rec.t = t;
    rec.set_face_normal(r, normal);
    rec.mat_ptr = make_shared<lambertian>(color(.65, .05, .05));

    return true;
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

BVHBuilder::bvhNodeInternal::bvhNodeInternal(
    const std::vector<shared_ptr<hittable>>& src_objects, size_t start, size_t end, double time0, double time1)
{
    std::vector<shared_ptr<hittable>> objects = src_objects; // Create a modifiable array of the source scene objects
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

        left = make_shared<bvhNodeInternal>(objects, start, mid, time0, time1);
        right = make_shared<bvhNodeInternal>(objects, mid, end, time0, time1);
    }

    aabb box_left, box_right;

    if (!left->bounding_box(time0, time1, box_left) || !right->bounding_box(time0, time1, box_right))
        std::cerr << "No bounding box in bvh_node constructor.\n";

    box = surrounding_box(box_left, box_right);
}
