#include "modelloader.h"

const std::string PATH = "misc/cornell_box/cornell_box.gltf"; //"misc/cornell_box/cornell_box.gltf";

hittable_list triangles()
{
    hittable_list world;
    loadModelGltf(PATH, world);

    auto red = make_shared<lambertian>(color(.65, .05, .05));
    auto white = make_shared<lambertian>(color(.73, .73, .73));
    auto green = make_shared<lambertian>(color(.12, .45, .15));

    auto checker = make_shared<checker_texture>(color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));

    hittable_list objects;
    objects.add(make_shared<bvh_node>(world, 0, 1, 0));

    /* for (int i = 0; i < bvh.size(); ++i)
        std::cout << i << ":" << std::endl
                  << bvh[i].m_minBounds << std::endl
                  << bvh[i].m_instanceIndex << std::endl
                  << bvh[i].m_maxBounds << std::endl
                  << bvh[i].m_nodeOffset << std::endl; */

    return objects;
}

color ray_color(const ray& r, const hittable& world, int depth)
{
    hit_record rec;

    // If we've exceeded the ray bounce limit, no more light is gathered.
    if (depth <= 0)
        return color(0, 0, 0);

    //(world.hit(r, 0.001, infinity, rec))
    // (hitAny(r, rec))
    if (world.hit(r, 0.001, infinity, rec))
    {
        ray scattered;
        color attenuation;
        if (rec.mat_ptr->scatter(r, rec, attenuation, scattered))
            return attenuation * ray_color(scattered, world, depth - 1);
        return color(0, 0, 0);
    }
    vec3 unit_direction = unit_vector(r.direction());
    auto t = 0.5 * (unit_direction.y() + 1.0);
    return (1.0 - t) * color(1.0, 1.0, 1.0) + t * color(0.5, 0.7, 1.0);
}

int main()
{
    // Image
    auto aspect_ratio = 16.0 / 9.0;
    int image_width = 400;
    int samples_per_pixel = 10;
    const int max_depth = 100;

    hittable_list world;

    point3 lookfrom;
    point3 lookat;
    auto vfov = 40.0;
    auto aperture = 0.0;

    world = triangles();
    lookfrom = point3(0.0, 3.0, 4.0);
    lookat = point3(0.0, 0.0, 0.0);
    vfov = 50.0;

    // Camera
    vec3 vup(0, 1, 0);
    auto dist_to_focus = 10.0;
    int image_height = static_cast<int>(image_width / aspect_ratio);

    camera cam(lookfrom, lookat, vup, vfov, aspect_ratio, aperture, dist_to_focus, 0.0, 1.0);

    std::vector<int8_t> colorData;

    colorData.resize(image_height * image_width * 3);
    int index = 0;

    for (int j = image_height - 1; j >= 0; --j)
    {
        std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
        for (int i = 0; i < image_width; ++i)
        {
            color pixel_color(0, 0, 0);
            for (int s = 0; s < samples_per_pixel; ++s)
            {
                auto u = (i + random_double()) / (image_width - 1);
                auto v = (j + random_double()) / (image_height - 1);
                ray r = cam.get_ray(u, v);
                pixel_color += ray_color(r, world, max_depth);
            }

            auto r = pixel_color.x();
            auto g = pixel_color.y();
            auto b = pixel_color.z();

            // Divide the color by the number of samples and gamma-correct for gamma=2.0.
            auto scale = 1.0 / samples_per_pixel;
            r = sqrt(scale * r);
            g = sqrt(scale * g);
            b = sqrt(scale * b);

            int ir = static_cast<int>(256 * clamp(r, 0.0, 0.999)), ig = static_cast<int>(256 * clamp(g, 0.0, 0.999)),
                ib = static_cast<int>(256 * clamp(b, 0.0, 0.999));

            colorData[index++] = ir;
            colorData[index++] = ig;
            colorData[index++] = ib;
        }
    }

    stbi_write_jpg("result.jpg", image_width, image_height, 3, colorData.data(), 3 * image_width);
}