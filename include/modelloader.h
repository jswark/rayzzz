#pragma once

#include "bvh.h"
#include "camera.h"

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

    std::vector<glm::vec3> vertices;
    vertices.reserve(vertexCount);
    for (uint32_t v = 0; v < vertexCount; ++v)
    {
        vertices.push_back(glm::make_vec3(&positionData[v * posStride]) * globalScale);
    }
    uint32_t indexCount = 0;
    std::vector<uint32_t> indices;
    const bool hasIndices = (primitive.indices != -1);
    assert(hasIndices); // currently support only this mode
    if (hasIndices)
    {
        const tinygltf::Accessor& accessor = model.accessors[primitive.indices > -1 ? primitive.indices : 0];
        const tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
        const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];

        indexCount = static_cast<uint32_t>(accessor.count);
        assert(indexCount != 0 && (indexCount % 3 == 0));
        const void* dataPtr = &(buffer.data[accessor.byteOffset + bufferView.byteOffset]);

        indices.reserve(indexCount);

        switch (accessor.componentType)
        {
        case TINYGLTF_PARAMETER_TYPE_UNSIGNED_INT: {
            const uint32_t* buf = static_cast<const uint32_t*>(dataPtr);
            for (size_t index = 0; index < indexCount; index++)
            {
                indices.push_back(buf[index]);
            }
            break;
        }
        case TINYGLTF_PARAMETER_TYPE_UNSIGNED_SHORT: {
            const uint16_t* buf = static_cast<const uint16_t*>(dataPtr);
            for (size_t index = 0; index < indexCount; index++)
            {
                indices.push_back(buf[index]);
            }
            break;
        }
        case TINYGLTF_PARAMETER_TYPE_UNSIGNED_BYTE: {
            const uint8_t* buf = static_cast<const uint8_t*>(dataPtr);
            for (size_t index = 0; index < indexCount; index++)
            {
                indices.push_back(buf[index]);
            }
            break;
        }
        default:
            std::cerr << "Index component type " << accessor.componentType << " not supported!" << std::endl;
            return;
        }
    }

    world.add(vertices, indices, transform);
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
            const float floatRotation[4] = { (float)node.rotation[3], (float)node.rotation[0], (float)node.rotation[1],
                                             (float)node.rotation[2] };

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
