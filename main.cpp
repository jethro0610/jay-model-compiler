#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/gtx/quaternion.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include "buffer.hpp"
#include "definitions.hpp"

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <tiny_gltf.h>

namespace gltf = tinygltf;
using namespace glm;

struct gltfMeshData {
    tinygltf::Mesh mesh;
    mat4 worldMatrix;
    std::string name;
};

SkeletalVertex StaticVertexToSkeletal(StaticVertex staticVertex) {
    SkeletalVertex skeletalVertex;
    skeletalVertex.position = staticVertex.position;
    skeletalVertex.normal = staticVertex.normal;
    skeletalVertex.tangent = staticVertex.tangent;
    skeletalVertex.bitangent = staticVertex.bitangent;
    skeletalVertex.uv = staticVertex.uv;
    return skeletalVertex;
}

typedef std::pair<gltf::Node, mat4> NodeAndMatrix;
bool CompareNodePair(const NodeAndMatrix& a, const NodeAndMatrix& b) {
    return a.first.name < b.first.name;
}

int ExitPrompt(int exitCode, bool shouldPrompt) {
    if (shouldPrompt) {
        std::cout << "Press ENTER to close\n";
        std::cin.ignore();
    }
    return exitCode; 
}

int main(int argc, char* argv[]) {
    gltf::Model gltfModel;
    gltf::TinyGLTF loader;
    std::string err;
    std::string warn;

    // Determine if an exit prompt should be used
    bool shouldPrompt = true;
    if (argc >= 4) {
        if (std::string(argv[3]) == "noprompt")
            shouldPrompt = false;
    }

    // No file was given
    if (argc < 2) { 
        std::cout << "Error: missing input file\n";
        return ExitPrompt(-1, shouldPrompt);
    }

    // Load the gltf model
    std::string path = argv[1];
    bool isLoaded = loader.LoadASCIIFromFile(&gltfModel, &err, &warn, path); 
    if (!isLoaded) {
        std::cout << "Error: " << err << '\n';
        return ExitPrompt(-1, shouldPrompt);
    }
    std::cout << "Loaded file \"" << path << "\"\n";

    // Determine the output path of the file
    std::string outPath;
    if (argc >= 3)
        outPath = argv[2];
    else {
        outPath = path.substr(0, path.size() - 4) + ".jmd";
    }

    // Create the file
    std::ofstream file;
    file.open(outPath, std::ios::out | std::ios::binary);
    if (!file.is_open()) {
        std::cout << "Error: Failed to create new JMD file\n";
        return ExitPrompt(-1, shouldPrompt);
    }

    // Determine the transform of each mesh and its gltf mesh,
    // this is stored as a pair so we can iterate and relate the two
    std::vector<NodeAndMatrix> meshNodes;
    for (gltf::Node node : gltfModel.nodes) {
        int meshIndex = node.mesh;
        if (meshIndex == -1)
            continue;

        vec3 position = vec3(0.0f);
        vec3 wScale = vec3(1.0f);
        quat rotation = quat(vec3(0.0f));

        for (int i = 0; i < node.translation.size(); i++)
            position[i] = node.translation[i];
        for (int i = 0; i < node.scale.size(); i++)
            wScale[i] = node.scale[i];
        for (int i = 0; i < node.rotation.size(); i++)
            rotation[i] = node.rotation[i];

        mat4 worldMatrix = mat4(1.0f);
        worldMatrix = translate(worldMatrix, position);
        worldMatrix *= toMat4(rotation); 
        worldMatrix = scale(worldMatrix, wScale);

        meshNodes.push_back(NodeAndMatrix(node, worldMatrix));
    }
    std::sort(meshNodes.begin(), meshNodes.end(), CompareNodePair);

    // Write the model header
    ModelHeader modelHeader;
    bool skeletal = false;
    modelHeader.numMeshes = meshNodes.size();
    modelHeader.numAnimations = 0;
    modelHeader.skeletal = false;
    if (gltfModel.skins.size() != 0) { 
        skeletal = true;
        modelHeader.skeletal = true;
        modelHeader.numAnimations = gltfModel.animations.size();
    }
    file.write((const char*)&modelHeader, sizeof(ModelHeader));

    std::cout << "Compiling model with " << modelHeader.numMeshes << " meshes\n";

    for (NodeAndMatrix nodeAndMatrix : meshNodes) {
        gltf::Mesh& gltfMesh= gltfModel.meshes[nodeAndMatrix.first.mesh];
        mat4 worldMatrix = nodeAndMatrix.second;
        mat3 normalMatrix = transpose(inverse(mat3(worldMatrix)));

        // Get the meshes buffers
        Buffer<vec3> positionBuf(gltfModel, gltfMesh, "POSITION");
        Buffer<vec3> normalBuf(gltfModel, gltfMesh, "NORMAL");
        Buffer<vec4> tangentBuf(gltfModel, gltfMesh, "TANGENT");
        Buffer<vec2> uvBuf(gltfModel, gltfMesh, "TEXCOORD_0");
        Buffer<uint16_t> indicesBuf(gltfModel, gltfMesh, "INDICES");

        // Get the skeletal buffers
        Buffer<u8vec4> jointBuf = Buffer<u8vec4>(gltfModel, gltfMesh, skeletal ? "JOINTS_0" : "INDICES");
        Buffer<vec4> weightBuf = Buffer<vec4>(gltfModel, gltfMesh, skeletal ? "WEIGHTS_0" : "INDICES");

        // Write the mesh header
        MeshHeader meshHeader;
        meshHeader.numVertices = positionBuf.size();
        meshHeader.numIndices = indicesBuf.size();
        file.write((const char*)&meshHeader, sizeof(MeshHeader));
        std::cout << 
            "\tCompiling mesh " <<
            nodeAndMatrix.first.name << 
            " with " << 
            meshHeader.numVertices << 
            " vertices and " << 
            meshHeader.numIndices << 
            " indices\n"; 

        // Ensure the buffer sizes match, otherwise the mesh
        // is invalid or something in the compiler is wrong
        if (
            normalBuf.size() != positionBuf.size() ||
            tangentBuf.size() != positionBuf.size() ||
            uvBuf.size() != positionBuf.size()
        ) {
            std::cout << "\tError: buffer sizes don't match\n";
            return ExitPrompt(-1, shouldPrompt);
        }

        // Write each vertex of the mesh one-by-one
        // std::cout << "\t\tWriting vertices\n";
        for (int i = 0; i < positionBuf.size(); i++) {
            StaticVertex vertex;
            vertex.position = worldMatrix * vec4(positionBuf[i], 1.0f);
            vertex.normal = normalize(normalMatrix * normalBuf[i]);
            vec4 tan4 = tangentBuf[i];
            vertex.tangent = normalize(normalMatrix * vec3(tan4));
            vertex.bitangent = normalize(cross(vertex.normal, vertex.tangent) * tan4.w);
            vertex.uv = uvBuf[i];

            // Write the skeletal data
            if (skeletal) {
                SkeletalVertex skeletalVertex = StaticVertexToSkeletal(vertex);
                skeletalVertex.joints = jointBuf[i];
                skeletalVertex.weights = weightBuf[i];
                file.write((const char*)&skeletalVertex, sizeof(SkeletalVertex));
            }
            else
                file.write((const char*)&vertex, sizeof(StaticVertex));
        }

        // Write the indices buffer of the mesh
        // std::cout << "\t\tWriting indices\n";
        file.write((const char*)indicesBuf.data(), sizeof(uint16_t) * indicesBuf.size());
    }
    std::cout << '\n';

    // If there's no skeleton, then we can stop writing to the model
    if (!skeletal) {
        file.close();
        std::cout << "Finished compiling static model to file \"" << outPath << "\"\n";
        return ExitPrompt(0, shouldPrompt);
    }

    std::cout << "Compiling skeleton joints\n";
    int numJoints = gltfModel.skins[0].joints.size();
    gltf::Accessor ibmAccessor = gltfModel.accessors[gltfModel.skins[0].inverseBindMatrices];
    Buffer<mat4> ibmBuffer(gltfModel, ibmAccessor);
    assert(ibmBuffer.size() == numJoints);

    // Children of joints are stored as nodes, but we need
    // their actual joint index to bind properly. Since node
    // and joint indices can be different, a map is necessary
    // to know which node corresponds to which joint
    std::map<int, int> nodeIndexToJointIndex;
    for (int i = 0; i < numJoints; i++)
        nodeIndexToJointIndex[gltfModel.skins[0].joints[i]] = i; 

    Skeleton skeleton;
    for (int j = 0; j < numJoints; j++) {
        // Get the node corresponding to the joint
        gltf::Node node = gltfModel.nodes[gltfModel.skins[0].joints[j]];
        Joint joint; 
        std::cout << "\tCompiling joint "  << node.name;
        if (node.children.size() > 0)
            std::cout << " with children:\n";
        else
            std::cout << '\n';

        // Copy the joint transform
        for (int i = 0; i < node.translation.size(); i++)
            joint.transform.position[i] = node.translation[i];
        for (int i = 0; i < node.rotation.size(); i++)
            joint.transform.rotation[i] = node.rotation[i];
        for (int i = 0; i < node.scale.size(); i++)
            joint.transform.scale[i] = node.scale[i];

        // Copy the inverse bind matrix
        joint.inverseBindMatrix = ibmBuffer[j];

        // Copy the joint's children, this is where the node
        // to joint conversion occurs
        assert(node.children.size() <= MAX_JOINT_CHILDREN);
        for (int i = 0; i < node.children.size(); i++) {
            joint.children.push_back(nodeIndexToJointIndex[node.children[i]]);
            std::cout << "\t\t" << gltfModel.nodes[node.children[i]].name << '\n';
        }
        skeleton.joints.push_back(joint);
    }
    file.write((const char*)&skeleton, sizeof(Skeleton));
    std::cout << '\n';

    std::cout << "Compiling animations\n";
    for (gltf::Animation gltfAnim : gltfModel.animations) {
        // Get the time and number of keyframes for the animation
        Buffer<float> timeBuffer(gltfModel, gltfModel.accessors[gltfAnim.samplers[0].input]);
        Animation animation;
        animation.keyframes.resize(timeBuffer.size());

        // Write the animation header
        AnimationHeader animHeader;
        animHeader.numKeyframes = animation.keyframes.size();
        strcpy_s(animHeader.name, gltfAnim.name.c_str());
        file.write((const char*)&animHeader, sizeof(AnimationHeader));
        std::cout << "\tCompiling animation " << animHeader.name << " with " << animHeader.numKeyframes << " keyframes\n";

        // Copy the keyframe times and size the transforms to the number of joints
        for (int i = 0; i < animation.keyframes.size(); i++) { 
            animation.keyframes[i].time = timeBuffer[i];
            animation.keyframes[i].transforms.resize(numJoints);
        }

        // Create the keyframes for every joint
        for (int i = 0; i < numJoints; i++) {
            gltf::AnimationChannel posChannel = gltfAnim.channels[i * 3 + 0]; 
            gltf::AnimationSampler posSampler = gltfAnim.samplers[posChannel.sampler];
            Buffer<vec4> posBuffer(gltfModel, gltfModel.accessors[posSampler.output]);
            int targetJoint = nodeIndexToJointIndex[posChannel.target_node];

            gltf::AnimationChannel rotChannel = gltfAnim.channels[i * 3 + 1]; 
            gltf::AnimationSampler rotSampler = gltfAnim.samplers[rotChannel.sampler];
            Buffer<quat> rotBuffer(gltfModel, gltfModel.accessors[rotSampler.output]);

            gltf::AnimationChannel scaleChannel = gltfAnim.channels[i * 3 + 2]; 
            gltf::AnimationSampler scaleSampler = gltfAnim.samplers[posChannel.sampler];
            Buffer<vec3> scaleBuffer(gltfModel, gltfModel.accessors[scaleSampler.output]);

            for (int k = 0; k < animation.keyframes.size(); k++) {
                Transform transform;
                transform.position = posBuffer[k];
                transform.rotation = rotBuffer[k];
                transform.scale = scaleBuffer[k];
                animation.keyframes[k].transforms[targetJoint] = transform;
            }
        }

        // Write the keyframees to the file
        for (Keyframe& keyframe : animation.keyframes)
            file.write((const char*)&keyframe, sizeof(Keyframe));
    }
    std::cout << '\n';

    file.close();
    std::cout << "Finished compiling skeletal model to file \"" << outPath << "\"\n";
    return ExitPrompt(0, shouldPrompt);
}
