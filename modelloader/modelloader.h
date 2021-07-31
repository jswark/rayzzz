#pragma once

#include <string>
#include <vector>


class ModelLoader
{
public:
    explicit ModelLoader()= default;;

    bool loadModelGltf(const std::string& modelPath, hittable_list& world);
};
