#include "rtweekend.h"
#include <iostream>

#include "bvh.h"
#include "camera.h"
#include "color.h"
#include "hittablelist.h"
#include "material.h"
#include "triangle.h"

hittable_list triangles() {
  hittable_list world;

  auto red = make_shared<lambertian>(color(.65, .05, .05));
  auto white = make_shared<lambertian>(color(.73, .73, .73));
  auto green = make_shared<lambertian>(color(.12, .45, .15));

  auto checker =
      make_shared<checker_texture>(color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));
  vec3 p1(0.0, 0.0, 0.0), p2(0.0, 0.0, -1.0), p3(0.0, 1.0, 0.0),
      p4(0.0, 1.0, -1.0), p5(1.0, 0.0, 0.0), p6(1.0, 0.0, -1.0),
      p7(1.0, 1.0, 0.0), p8(1.0, 1.0, -1.0);

  // very random scene, hundred apologises for the awful view
  world.add(make_shared<triangle>(vec3(0, 0, 0.5), vec3(-1, 1, 0),
                                  vec3(-1, -1, 0), red, 0));

  hittable_list objects;

  world.add(make_shared<triangle>(p1, p4, p2, red, 1));

  world.add(make_shared<triangle>(vec3(-1.27, 0.19, 1),
                                  vec3(-0.43, -0.43, 0.28), vec3(-1, -0.34, 0),
                                  white, 2));
  world.add(make_shared<triangle>(p3, p7, p8, white, 3));
  world.add(make_shared<triangle>(p1, p5, p6, white, 4));

  objects.add(make_shared<bvh_node>(world, 0, 1, 0));

 /* for (int i = 0; i < bvh.size(); ++i)
    std::cout << i << ":" << std::endl
              << bvh[i].m_minBounds << std::endl
              << bvh[i].m_instanceIndex << std::endl
              << bvh[i].m_maxBounds << std::endl
              << bvh[i].m_nodeOffset << std::endl; */

  return objects;
}

color ray_color(const ray &r, const hittable &world, int depth) {
  hit_record rec;

  // If we've exceeded the ray bounce limit, no more light is gathered.
  if (depth <= 0)
    return color(0, 0, 0);

  if (world.hit(r, 0.001, infinity, rec)) {
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

int main() {

  // Image
  auto aspect_ratio = 16.0 / 9.0;
  int image_width = 400;
  int samples_per_pixel = 400;
  const int max_depth = 50;

  hittable_list world;

  point3 lookfrom;
  point3 lookat;
  auto vfov = 40.0;
  auto aperture = 0.0;

  world = triangles();
  lookfrom = point3(-10, 0, 13);
  lookat = point3(-0.5, 0, 0);
  vfov = 20.0;

  // Camera

  vec3 vup(0, 1, 0);
  auto dist_to_focus = 10.0;
  int image_height = static_cast<int>(image_width / aspect_ratio);

  camera cam(lookfrom, lookat, vup, vfov, aspect_ratio, aperture, dist_to_focus,
             0.0, 1.0);

  std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

  for (int j = image_height - 1; j >= 0; --j) {
    std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
    for (int i = 0; i < image_width; ++i) {
      color pixel_color(0, 0, 0);
      for (int s = 0; s < samples_per_pixel; ++s) {
        auto u = (i + random_double()) / (image_width - 1);
        auto v = (j + random_double()) / (image_height - 1);
        ray r = cam.get_ray(u, v);
        pixel_color += ray_color(r, world, max_depth);
      }
      write_color(std::cout, pixel_color, samples_per_pixel);
    }
  }
  std::cerr << "\nDone.\n";
}