#pragma once

#include "aabb.h"
#include "hittable.h"
#include "material.h"

#include <memory>
#include <vector>

using std::make_shared;
using std::shared_ptr;

#include "triangle.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/gtx/matrix_decompose.hpp>

class hittable_list : public hittable
{
public:
    hittable_list()
    {
    }

    void clear()
    {
        objects.clear();
    }
    
    void add(const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& indices, const glm::float4x4& transform)
    {
        auto red = make_shared<lambertian>(color(.65, .05, .05));

        for (uint32_t i = 0; i < indices.size(); i += 3)
        {
            uint32_t i0 = indices[i + 0];
            uint32_t i1 = indices[i + 1];
            uint32_t i2 = indices[i + 2];

            glm::vec4 v0t = transform * glm::vec4{ vertices[i0], 1.0 };
            glm::vec4 v1t = transform * glm::vec4{ vertices[i1], 1.0 };
            glm::vec4 v2t = transform * glm::vec4{ vertices[i2], 1.0 };

            glm::vec3 v0 = glm::vec3{ v0t.x, v0t.y, v0t.z } / v0t.w;
            glm::vec3 v1 = glm::vec3{ v1t.x, v1t.y, v1t.z } / v1t.w;
            glm::vec3 v2 = glm::vec3{ v2t.x, v2t.y, v2t.z } / v2t.w;

            point3 p0 = { v0.x, v0.y, v0.z };
            point3 p1 = { v1.x, v1.y, v1.z };
            point3 p2 = { v2.x, v2.y, v2.z };

            objects.push_back(make_shared<triangle>(p0, p1, p2, red, 0));
        }
    }

    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override;
    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;
    virtual std::vector<point3> getCoord() const override
    {
        return {};
    };
    virtual int getIndex() const override
    {
        return {};
    };
    virtual void reconstruct(int index) override{};

public:
    std::vector<shared_ptr<hittable>> objects;
};

bool hittable_list::hit(const ray& r, double t_min, double t_max, hit_record& rec) const
{
    hit_record temp_rec;
    bool hit_anything = false;
    auto closest_so_far = t_max;

    for (const auto& object : objects)
    {
        if (object->hit(r, t_min, closest_so_far, temp_rec))
        {
            hit_anything = true;
            closest_so_far = temp_rec.t;
            rec = temp_rec;
        }
    }

    return hit_anything;
}

bool hittable_list::bounding_box(double time0, double time1, aabb& output_box) const
{
    if (objects.empty())
        return false;

    aabb temp_box;
    bool first_box = true;

    for (const auto& object : objects)
    {
        if (!object->bounding_box(time0, time1, temp_box))
            return false;
        output_box = first_box ? temp_box : surrounding_box(output_box, temp_box);
        first_box = false;
    }

    return true;
}
