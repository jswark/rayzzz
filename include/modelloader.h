#pragma once

#include "bvh.h"
#include "camera.h"
#include "material.h"

#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#define TINYGLTF_IMPLEMENTATION

#include "triangle.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/gtx/matrix_decompose.hpp>

#include <tiny_gltf.h>

void processPrimitive(const tinygltf::Model& model,
                      const tinygltf::Primitive& primitive,
                      const glm::float4x4& transform,
                      const float globalScale,
                      hittable_list& world)
{
    using namespace std;
    assert(primitive.attributes.find("POSITION") != primitive.attributes.end());

    const tinygltf::Accessor& positionAccessor = model.accessors[primitive.attributes.find("POSITION")->second];
    const tinygltf::BufferView& positionView = model.bufferViews[positionAccessor.bufferView];
    const float* positionData = reinterpret_cast<const float*>(
        &model.buffers[positionView.buffer].data[positionAccessor.byteOffset + positionView.byteOffset]);
    assert(positionData != nullptr);
    const uint32_t vertexCount = static_cast<uint32_t>(positionAccessor.count);
    assert(vertexCount != 0);
    const int byteStride = positionAccessor.ByteStride(positionView);
    assert(byteStride > 0); // -1 means invalid glTF
    int posStride = byteStride / sizeof(float);

    uint32_t currVertexNumber = 0;
    auto red = make_shared<lambertian>(color(.65, .05, .05));
    std::vector<glm::vec3> currTriangle;
    for (uint32_t v = 0; v < vertexCount; ++v)
    {
        ++currVertexNumber;
        glm::vec3 pos = glm::make_vec3(&positionData[v * posStride]) * globalScale;
        currTriangle.push_back(pos);

        if (currVertexNumber == 3)
        {
            world.add(make_shared<triangle>(vec3(currTriangle[0].x, currTriangle[0].y, currTriangle[0].z),
                                            vec3(currTriangle[1].x, currTriangle[1].y, currTriangle[1].z),
                                            vec3(currTriangle[2].x, currTriangle[2].y, currTriangle[2].z), red, 0));

            std::cout
                << "(" << currTriangle[0].x << "," << currTriangle[0].y << "," << currTriangle[0].z << ")" << std::endl
                << "(" << currTriangle[1].x << "," << currTriangle[1].y << "," << currTriangle[1].z << ")" << std::endl
                << "(" << currTriangle[2].x << "," << currTriangle[2].y << "," << currTriangle[2].z << ")" << std::endl
                << std::endl;

            currTriangle.clear();
            currVertexNumber = 0;
        }
    }
}

void processMesh(const tinygltf::Model& model,
                 const tinygltf::Mesh& mesh,
                 const glm::float4x4& transform,
                 const float globalScale,
                 hittable_list& world)
{
    using namespace std;
    cout << "Mesh name: " << mesh.name << endl;
    cout << "Primitive count: " << mesh.primitives.size() << endl;
    for (size_t i = 0; i < mesh.primitives.size(); ++i)
    {
        processPrimitive(model, mesh.primitives[i], transform, globalScale, world);
    }
}

glm::float4x4 getTransform(const tinygltf::Node& node, const float globalScale)
{
    if (node.matrix.empty())
    {
        glm::float3 scale{ 1.0f };
        if (!node.scale.empty())
        {
            scale = glm::float3((float)node.scale[0], (float)node.scale[1], (float)node.scale[2]);
            // check that scale is uniform, otherwise we have to support it in shader
            // assert(scale.x == scale.y && scale.y == scale.z);
        }

        glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        if (!node.rotation.empty())
        {
            const float floatRotation[4] = { (float)node.rotation[0], (float)node.rotation[1], (float)node.rotation[2],
                                             (float)node.rotation[3] };
            rotation = glm::make_quat(floatRotation);
        }

        glm::float3 translation{ 0.0f };
        if (!node.translation.empty())
        {
            translation = glm::float3((float)node.translation[0], (float)node.translation[1], (float)node.translation[2]);
            translation *= globalScale;
        }

        const glm::float4x4 translationMatrix = glm::translate(glm::float4x4(1.0f), translation);
        const glm::float4x4 rotationMatrix{ rotation };
        const glm::float4x4 scaleMatrix = glm::scale(glm::float4x4(1.0f), scale);

        const glm::float4x4 localTransform = translationMatrix * rotationMatrix * scaleMatrix;

        return localTransform;
    }
    else
    {
        glm::float4x4 localTransform = glm::make_mat4(node.matrix.data());
        return localTransform;
    }
}

void processNode(const tinygltf::Model& model,
                 const tinygltf::Node& node,
                 const glm::float4x4& baseTransform,
                 const float globalScale,
                 hittable_list& world)
{
    using namespace std;
    cout << "Node name: " << node.name << endl;

    const glm::float4x4 localTransform = getTransform(node, globalScale);
    const glm::float4x4 globalTransform = baseTransform * localTransform;

    if (node.mesh != -1) // mesh exist
    {
        const tinygltf::Mesh& mesh = model.meshes[node.mesh];
        processMesh(model, mesh, globalTransform, globalScale, world);
    }
    else if (node.camera != -1) // camera node
    {
        glm::vec3 scale;
        glm::quat rotation;
        glm::vec3 translation;
        glm::vec3 skew;
        glm::vec4 perspective;
        glm::decompose(globalTransform, scale, rotation, translation, skew, perspective);
        // scene.getCamera(node.camera).position = translation;
        // scene.getCamera(node.camera).mOrientation = rotation;
        // scene.getCamera(node.camera).updateViewMatrix();
    }

    for (int i = 0; i < node.children.size(); ++i)
    {
        processNode(model, model.nodes[node.children[i]], globalTransform, globalScale, world);
    }
}

bool loadModelGltf(const std::string& modelPath, hittable_list& world)
{
    if (modelPath.empty())
    {
        return false;
    }

    using namespace std;
    tinygltf::Model model;
    tinygltf::TinyGLTF gltf_ctx;
    std::string err;
    std::string warn;
    bool res = gltf_ctx.LoadASCIIFromFile(&model, &err, &warn, modelPath.c_str());
    if (!res)
    {
        cerr << "Unable to load file: " << modelPath << endl;
        return res;
    }
    for (int i = 0; i < model.scenes.size(); ++i)
    {
        cout << "Scene: " << model.scenes[i].name << endl;
    }

    int sceneId = model.defaultScene;

    const float globalScale = 1.0f;

    for (int i = 0; i < model.scenes[sceneId].nodes.size(); ++i)
    {
        const int rootNodeIdx = model.scenes[sceneId].nodes[i];
        processNode(model, model.nodes[rootNodeIdx], glm::float4x4(1.0f), globalScale, world);
    }
    return res;
}