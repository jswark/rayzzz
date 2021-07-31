#pragma once

#include "bvh.h"
#include "camera.h"
#include "material.h"

#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#define TINYGLTF_IMPLEMENTATION

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/gtx/matrix_decompose.hpp>

#include "triangle.h"
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

    // Normals
    const float* normalsData = nullptr;
    int normalStride = 0;
    if (primitive.attributes.find("NORMAL") != primitive.attributes.end())
    {
        const tinygltf::Accessor& normalAccessor = model.accessors[primitive.attributes.find("NORMAL")->second];
        const tinygltf::BufferView& normView = model.bufferViews[normalAccessor.bufferView];
        normalsData = reinterpret_cast<const float*>(
            &(model.buffers[normView.buffer].data[normalAccessor.byteOffset + normView.byteOffset]));
        assert(normalsData != nullptr);
        normalStride = normalAccessor.ByteStride(normView) / sizeof(float);
        assert(normalStride > 0);
    }

    // UVs
    const float* texCoord0Data = nullptr;
    int texCoord0Stride = 0;
    if (primitive.attributes.find("TEXCOORD_0") != primitive.attributes.end())
    {
        const tinygltf::Accessor& uvAccessor = model.accessors[primitive.attributes.find("TEXCOORD_0")->second];
        const tinygltf::BufferView& uvView = model.bufferViews[uvAccessor.bufferView];
        texCoord0Data = reinterpret_cast<const float*>(
            &(model.buffers[uvView.buffer].data[uvAccessor.byteOffset + uvView.byteOffset]));
        texCoord0Stride = uvAccessor.ByteStride(uvView) / sizeof(float);
    }

    int matId = primitive.material;
    if (matId == -1)
    {
        matId = 0; // TODO: should be index of default material
    }

    glm::float3 sum = glm::float3(0.0f, 0.0f, 0.0f);
    // std::vector<nevk::Scene::Vertex> vertices;
    // vertices.reserve(vertexCount);
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
            currTriangle.clear();
            currVertexNumber = 0;
        }
        // vertex.normal = packNormal(
        //     glm::normalize(glm::vec3(normalsData ? glm::make_vec3(&normalsData[v * normalStride]) :
        //     glm::vec3(0.0f))));
        // vertex.uv = packUV(texCoord0Data ? glm::make_vec2(&texCoord0Data[v * texCoord0Stride]) : glm::vec3(0.0f));
        // vertices.push_back(vertex);
        // sum += vertex.pos;
    }
    const glm::float3 massCenter = sum / (float)vertexCount;

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

    // uint32_t meshId = scene.createMesh(vertices, indices);
    // assert(meshId != -1);
    // uint32_t instId = scene.createInstance(meshId, matId, transform, massCenter);
    // assert(instId != -1);
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
/*
void loadCameras(const tinygltf::Model& model)
{
    for (uint32_t i = 0; i < model.cameras.size(); ++i)
    {
        const tinygltf::Camera& cameraGltf = model.cameras[i];
        if (strcmp(cameraGltf.type.c_str(), "perspective") == 0)
        {
            nevk::Camera camera;
            camera.fov = cameraGltf.perspective.yfov * (180.0f / 3.1415926f);
            camera.znear = cameraGltf.perspective.znear;
            camera.zfar = cameraGltf.perspective.zfar;
            camera.name = cameraGltf.name;
            scene.addCamera(camera);
        }
        else
        {
            // not supported
        }
    }
    if (scene.getCameraCount() == 0)
    {
        // add default camera
        Camera camera;
        camera.updateViewMatrix();
        scene.addCamera(camera);
    }
}
*/

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

    // loadCameras(model);

    const float globalScale = 1.0f;

    for (int i = 0; i < model.scenes[sceneId].nodes.size(); ++i)
    {
        const int rootNodeIdx = model.scenes[sceneId].nodes[i];
        processNode(model, model.nodes[rootNodeIdx], glm::float4x4(1.0f), globalScale, world);
    }
    return res;
}