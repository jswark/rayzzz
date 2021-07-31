#pragma once

#include <string>
#include <vector>


class ModelLoader
{
private:
    TextureManager* mTexManager = nullptr;

public:
    explicit ModelLoader(nevk::TextureManager* texManager) : mTexManager(texManager){};

    bool loadModelGltf(const std::string& modelPath);
};
