#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <tiny_gltf.h>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/gtx/quaternion.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

using namespace tinygltf;
using namespace glm;

const int MAX_JOINT_CHILDREN = 8;

struct JTransform {
    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 scale;
};

struct JModelHeader {
    int numMeshes;
    int numJoints;
};

struct JMeshHeader {
    int numVertices;
    int numIndices;
};

struct JJoint {
    JTransform transform;
    glm::mat4 inverseBindMatrix;
    int children[MAX_JOINT_CHILDREN];
};

struct JBuffer {
    void* data;
    size_t count;
};

JBuffer GetBufferFromAccessor(Model& model, Accessor& accessor) {
    BufferView& view = model.bufferViews[accessor.bufferView];
    Buffer& buffer = model.buffers[view.buffer];

    int dataOffset = view.byteOffset + accessor.byteOffset;
    return {
        &buffer.data[dataOffset],
        accessor.count
    };
}

JBuffer GetBuffer(Model& model, Mesh& mesh, std::string attributeName) {
    Accessor accessor = model.accessors[mesh.primitives[0].attributes.at(attributeName)];
    return GetBufferFromAccessor(model, accessor);
};

JBuffer GetIndices(Model& model, Mesh& mesh) {
    Accessor accessor = model.accessors[mesh.primitives[0].indices];

    BufferView& view = model.bufferViews[accessor.bufferView];
    Buffer& buffer = model.buffers[view.buffer];

    int dataOffset = view.byteOffset + accessor.byteOffset;
    return {
        &buffer.data[dataOffset],
        accessor.count
    };
}

struct JStaticVertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 tangent;
    glm::vec3 bitangent;
    glm::vec2 uv;
};

struct JSkeletalVertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 tangent;
    glm::vec3 bitangent;
    glm::vec2 uv;
    glm::ivec4 joints;
    glm::vec4 weights;
};

JSkeletalVertex StaticVertexToSkeletal(JStaticVertex staticVertex) {
    JSkeletalVertex skeletalVertex;
    skeletalVertex.position = staticVertex.position;
    skeletalVertex.normal = staticVertex.normal;
    skeletalVertex.tangent = staticVertex.tangent;
    skeletalVertex.bitangent = staticVertex.bitangent;
    skeletalVertex.uv = staticVertex.uv;
    return skeletalVertex;
}

int exitPrompt(int exitCode, bool shouldPrompt) {
    if (shouldPrompt) {
        std::cout << "Press ENTER to close\n";
        std::cin.ignore();
    }
    return exitCode; 
}

int main(int argc, char* argv[]) {
    Model model;
    TinyGLTF loader;
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
        return exitPrompt(-1, shouldPrompt);
    }

    // Load the gltf model
    std::string path = argv[1];
    bool isLoaded = loader.LoadBinaryFromFile(&model, &err, &warn, path); 
    if (!isLoaded) {
        std::cout << "Error: " << err << '\n';
        return exitPrompt(-1, shouldPrompt);
    }
    std::cout << "Loaded file \"" << path << "\"\n";

    // Determine the output path of the file
    std::string outPath;
    if (argc >= 3)
        outPath = argv[2];
    else {
        outPath = path.substr(0, path.size() - 4);
        outPath += ".jmd";
    }

    // Create the file
    std::ofstream file;
    file.open(outPath, std::ios::out | std::ios::binary);
    if (!file.is_open()) {
        std::cout << "Error: Failed to create new JMD file\n";
        return exitPrompt(-1, shouldPrompt);
    }

    // Determine the transform of each mesh and its gltf mesh,
    // this is stored as a pair so we can iterate and relate the two
    typedef std::pair<Mesh, mat4> MeshAndMatrix;
    std::vector<MeshAndMatrix> meshes;
    for (Node node : model.nodes) {
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

        meshes.push_back(MeshAndMatrix(model.meshes[node.mesh], worldMatrix));
    }

    // Write the model header
    JModelHeader modelHeader;
    bool skeletal = false;
    modelHeader.numMeshes = meshes.size();
    modelHeader.numJoints = 0;
    if (model.skins.size() != 0) { 
        modelHeader.numJoints = model.skins[0].joints.size();
        skeletal = true;
    }
    file.write((const char*)&modelHeader, sizeof(JModelHeader));

    std::cout << "Compiling model with " << modelHeader.numMeshes << " meshes\n";

    for (MeshAndMatrix meshAndMatrix : meshes) {
        Mesh& mesh = meshAndMatrix.first;
        mat4 worldMatrix = meshAndMatrix.second;
        mat3 normalMatrix = transpose(inverse(mat3(worldMatrix)));

        // Get the meshes buffers
        JBuffer positionBuf = GetBuffer(model, mesh, "POSITION");
        JBuffer normalBuf = GetBuffer(model, mesh, "NORMAL");
        JBuffer tangentBuf = GetBuffer(model, mesh, "TANGENT");
        JBuffer uvBuf = GetBuffer(model, mesh, "TEXCOORD_0");
        JBuffer indicesBuf = GetIndices(model, mesh);

        // Get the skeletal buffers
        JBuffer jointBuf = skeletal ? GetBuffer(model, mesh, "JOINTS_0") : JBuffer();
        JBuffer weightBuf = skeletal ? GetBuffer(model, mesh, "WEIGHTS_0") : JBuffer();

        // Write the mesh header
        JMeshHeader meshHeader;
        meshHeader.numVertices = positionBuf.count;
        meshHeader.numIndices = indicesBuf.count;
        file.write((const char*)&meshHeader, sizeof(JMeshHeader));
        std::cout << 
            "\tCompiling mesh " <<
            mesh.name << 
            " with " << 
            meshHeader.numVertices << 
            " vertices and " << 
            meshHeader.numIndices << 
            " indices\n"; 

        // Ensure the buffer sizes match, otherwise the mesh
        // is invalid or something in the compiler is wrong
        if (
            normalBuf.count != positionBuf.count ||
            tangentBuf.count != positionBuf.count ||
            uvBuf.count != positionBuf.count
        ) {
            std::cout << "\tError: buffer sizes don't match\n";
            return exitPrompt(-1, shouldPrompt);
        }

        // Write each vertex of the mesh one-by-one
        std::cout << "\t\tWriting vertices\n";
        for (int i = 0; i < positionBuf.count; i++) {
            JStaticVertex vertex;
            vertex.position = worldMatrix * vec4(((vec3*)positionBuf.data)[i], 1.0f);
            vertex.normal = normalize(normalMatrix * ((vec3*)normalBuf.data)[i]);
            vec4 tan4 = ((vec4*)tangentBuf.data)[i];
            vertex.tangent = normalize(normalMatrix * vec3(tan4));
            vertex.bitangent = normalize(cross(vertex.normal, vertex.tangent) * tan4.w);
            vertex.uv = ((vec2*)uvBuf.data)[i];

            // Write the skeletal data
            if (skeletal) {
                JSkeletalVertex skeletalVertex = StaticVertexToSkeletal(vertex);

                // Since gltf stores joint indices as an unsigned byte integer, 
                // we need to convert the numbers to ivec4 comptatible format. 
                // Index j is the component of the joint vector then.
                for (int j = 0; j < 4; j++)
                    skeletalVertex.joints[j] = ((uint8_t*)jointBuf.data)[i * 4 + j];

                skeletalVertex.weights = ((vec4*)weightBuf.data)[i];
                file.write((const char*)&skeletalVertex, sizeof(JSkeletalVertex));
            }
            else
                file.write((const char*)&vertex, sizeof(JStaticVertex));
        }

        // Write the indices buffer of the mesh
        std::cout << "\t\tWriting indices\n";
        file.write((const char*)indicesBuf.data, sizeof(uint16_t) * indicesBuf.count);
    }

    // If there's no skeleton, then we can stop writing to the model
    if (modelHeader.numJoints == 0) {
        file.close();
        std::cout << "Finished compiling static model to file \"" << outPath << "\"\n";
        return exitPrompt(0, shouldPrompt);
    }

    std::cout << "Compiling skeleton joints\n";
    Accessor ibmAccessor = model.accessors[model.skins[0].inverseBindMatrices];
    JBuffer ibmBuffer = GetBufferFromAccessor(model, ibmAccessor);
    assert(ibmBuffer.count == modelHeader.numJoints);

    // Children of joints are stored as nodes, but we need
    // their actual joint index to bind properly. Since node
    // and joint indices can be different, a map is necessary
    // to know which node corresponds to which joint
    std::map<int, int> nodeIndexToJointIndex;
    for (int i = 0; i < modelHeader.numJoints; i++)
        nodeIndexToJointIndex[model.skins[0].joints[i]] = i; 

    for (int j = 0; j < modelHeader.numJoints; j++) {
        // Get the node corresponding to the joint
        Node node = model.nodes[model.skins[0].joints[j]];
        JJoint joint; 

        // Copy the joint transform
        for (int i = 0; i < node.translation.size(); i++)
            joint.transform.position[i] = node.translation[i];
        for (int i = 0; i < node.rotation.size(); i++)
            joint.transform.rotation[i] = node.rotation[i];
        for (int i = 0; i < node.scale.size(); i++)
            joint.transform.scale[i] = node.scale[i];

        // Copy the inverse bind matrix
        joint.inverseBindMatrix = ((mat4*)ibmBuffer.data)[j];

        // Copy the joint's children, this is where the node
        // to joint conversion occurs
        assert(node.children.size() <= MAX_JOINT_CHILDREN);
        for (int i = 0; i < node.children.size(); i++)
            joint.children[i] = nodeIndexToJointIndex[node.children[i]];

        std::cout << "\tWriting joint " << j << '\n';
        file.write((const char*)&joint, sizeof(joint));
    }

    file.close();
    std::cout << "Finished compiling skeletal model to file \"" << outPath << "\"\n";
    return exitPrompt(0, shouldPrompt);
}
