#pragma once

#include "hittablelist.h"
#include "tiny_gltf.h"

#include <string>
#include <vector>

class ModelLoader
{
public:
    ModelLoader() = default;

    bool loadModelGltf(const std::string& modelPath, hittable_list& world);
};
