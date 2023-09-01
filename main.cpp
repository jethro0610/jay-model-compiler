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
        Buffer<u8vec4> boneBuf = Buffer<u8vec4>(gltfModel, gltfMesh, skeletal ? "JOINTS_0" : "INDICES");
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
                skeletalVertex.bones = boneBuf[i];
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

    std::cout << "Compiling skeleton bones\n";
    int numBones = gltfModel.skins[0].joints.size();
    gltf::Accessor ibmAccessor = gltfModel.accessors[gltfModel.skins[0].inverseBindMatrices];
    Buffer<mat4> ibmBuffer(gltfModel, ibmAccessor);
    assert(ibmBuffer.size() == numBones);

    // Children of bones are stored as nodes, but we need
    // their actual bone index to bind properly. Since node
    // and bone indices can be different, a map is necessary
    // to know which node corresponds to which bone
    std::map<int, int> nodeIndexToBoneIndex;
    for (int i = 0; i < numBones; i++)
        nodeIndexToBoneIndex[gltfModel.skins[0].joints[i]] = i; 

    Bones bones;
    Ribbons ribbons;
    for (int j = 0; j < numBones; j++) {
        // Get the node corresponding to the bone
        gltf::Node node = gltfModel.nodes[gltfModel.skins[0].joints[j]];
        Bone bone; 
        std::cout << "\t[" << j << "] "<< " Compiling bone "  << node.name;
        if (node.children.size() > 0)
            std::cout << " with children:\n";
        else
            std::cout << '\n';

        // Copy the inverse bind matrix
        bone.inverseBindMatrix = ibmBuffer[j];

        // Copy the bone's children, this is where the node
        // to bone conversion occurs
        assert(node.children.size() <= MAX_BONE_CHILDREN);
        for (int child : node.children) {
            bone.children.push_back(nodeIndexToBoneIndex[child]);
            std::cout << "\t\t" << "[" << nodeIndexToBoneIndex[child] << "] " << gltfModel.nodes[child].name << '\n';
        }
        bones.push_back(bone);

        // Create a ribbon description if the bone is a ribbon head
        if (node.extras.Has("ribbon_length")) {
            RibbonDesc ribbon;
            ribbon.start = nodeIndexToBoneIndex[gltfModel.skins[0].joints[j]];
            ribbon.end = ribbon.start + node.extras.Get("ribbon_length").GetNumberAsInt() - 1;
            ribbon.returnSpeed = node.extras.Get("ribbon_return_speed").GetNumberAsDouble();
            ribbon.tailPower = node.extras.Get("ribbon_tail_power").GetNumberAsDouble();
            ribbon.tailRatio = node.extras.Get("ribbon_tail_ratio").GetNumberAsDouble();
            ribbons.push_back(ribbon);
        }
    }
    file.write((const char*)&bones, sizeof(Bones));
    file.write((const char*)&ribbons, sizeof(Ribbons));
    std::cout << "Skeleton has " << ribbons.size() << " ribbons\n\n";

    std::cout << "Compiling animations\n";
    for (gltf::Animation gltfAnim : gltfModel.animations) {
        // Get the time and number of keyframes for the animation
        Buffer<float> timeBuffer(gltfModel, gltfModel.accessors[gltfAnim.samplers[0].input]);
        Animation animation;
        if (gltfAnim.extras.Has("framerate"))
            animation.framerate = gltfAnim.extras.Get("framerate").GetNumberAsInt();
        else
            animation.framerate = 6;
        animation.keyframes.resize(timeBuffer.size());

        // Write the animation header
        AnimationHeader animHeader;
        animHeader.numKeyframes = animation.keyframes.size();
        animHeader.framerate = animation.framerate;
        strncpy(animHeader.name, gltfAnim.name.c_str(), MAX_ANIM_NAME);
        file.write((const char*)&animHeader, sizeof(AnimationHeader));
        std::cout << "\tCompiling animation " << animHeader.name << " with " << animHeader.numKeyframes << " keyframes\n";

        // Size the transforms to the number of bones
        for (int i = 0; i < animation.keyframes.size(); i++)
            animation.keyframes[i].pose.resize(numBones);

        // Create the keyframes for every bone
        for (int i = 0; i < numBones; i++) {
            gltf::AnimationChannel posChannel = gltfAnim.channels[i * 3 + 0]; 
            gltf::AnimationSampler posSampler = gltfAnim.samplers[posChannel.sampler];
            Buffer<vec3> posBuffer(gltfModel, gltfModel.accessors[posSampler.output]);
            int targetBone = nodeIndexToBoneIndex[posChannel.target_node];

            gltf::AnimationChannel rotChannel = gltfAnim.channels[i * 3 + 1]; 
            gltf::AnimationSampler rotSampler = gltfAnim.samplers[rotChannel.sampler];
            Buffer<quat> rotBuffer(gltfModel, gltfModel.accessors[rotSampler.output]);

            gltf::AnimationChannel scaleChannel = gltfAnim.channels[i * 3 + 2]; 
            gltf::AnimationSampler scaleSampler = gltfAnim.samplers[scaleChannel.sampler];
            Buffer<vec3> scaleBuffer(gltfModel, gltfModel.accessors[scaleSampler.output]);

            for (int k = 0; k < animation.keyframes.size(); k++) {
                Transform transform;
                transform.position = posBuffer[k];
                transform.rotation = rotBuffer[k];
                transform.scale = scaleBuffer[k];
                animation.keyframes[k].pose[targetBone] = transform;
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
