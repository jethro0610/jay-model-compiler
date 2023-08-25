#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vector>
#include <vector_const.h>

const int MAX_BONES = 32;
const int MAX_BONE_CHILDREN = 8;
const int MAX_ANIM_NAME = 32;

struct Transform {
    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 scale;

    Transform() {
        position = glm::vec3(0.0f, 0.0f, 0.0f);
        rotation = glm::quat(glm::vec3(0.0f, 0.0f, 0.0f));
        scale = glm::vec3(1.0f, 1.0f, 1.0f);
    }

    glm::mat4 GetWorldMatrix() const {
        glm::mat4 outWorld;
        outWorld = glm::mat4(1.0f);
        outWorld = translate(outWorld, position);
        outWorld *= toMat4(rotation);
        outWorld = glm::scale(outWorld, scale);

        return outWorld;
    }
};

struct ModelHeader {
    int numMeshes;
    int numAnimations;
    bool skeletal;
};

struct MeshHeader {
    int numVertices;
    int numIndices;
};

struct StaticVertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 tangent;
    glm::vec3 bitangent;
    glm::vec2 uv;
};

struct SkeletalVertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 tangent;
    glm::vec3 bitangent;
    glm::vec2 uv;
    glm::u8vec4 bones;
    glm::vec4 weights;
};

struct Bone {
    glm::mat4 inverseBindMatrix;
    vector_const<int, MAX_BONE_CHILDREN> children;
};
typedef vector_const<Bone, MAX_BONES> Bones;
typedef vector_const<Transform, MAX_BONES> Pose;

struct AnimationHeader {
    char name[MAX_ANIM_NAME];
    int numKeyframes;
};

struct Keyframe {
    Pose pose;
};

struct Animation {
    std::vector<Keyframe> keyframes;
};
