#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vector>
#include <vector_const.h>

const int MAX_JOINTS = 32;
const int MAX_JOINT_CHILDREN = 8;
const int MAX_ANIM_NAME = 32;

struct Transform {
    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 scale;
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
    glm::u8vec4 joints;
    glm::vec4 weights;
};

struct Joint {
    Transform transform;
    glm::mat4 inverseBindMatrix;
    vector_const<int, MAX_JOINT_CHILDREN> children;
};

struct Skeleton {
    vector_const<Joint, MAX_JOINTS> joints;
};

struct AnimationHeader {
    char name[MAX_ANIM_NAME];
    int numKeyframes;
};

struct Keyframe {
    float time;
    vector_const<Transform, MAX_JOINTS> transforms;
};

struct Animation {
    std::vector<Keyframe> keyframes;
};
