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

struct JModelHeader {
    int numMeshes;
};

struct JMeshHeader {
    int numVertices;
    int numIndices;
};

struct JBuffer {
    void* data;
    size_t count;
};

JBuffer GetBuffer(Model& model, Mesh& mesh, std::string attributeName) {
    Accessor accessor = model.accessors[mesh.primitives[0].attributes.at(attributeName)];

    BufferView& view = model.bufferViews[accessor.bufferView];
    Buffer& buffer = model.buffers[view.buffer];

    int dataOffset = view.byteOffset + accessor.byteOffset;
    return {
        &buffer.data[dataOffset],
        accessor.count
    };
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

    bool shouldPrompt = true;
    if (argc >= 4) {
        if (std::string(argv[3]) == "noprompt")
            shouldPrompt = false;
    }

    if (argc < 2) { 
        std::cout << "Error: missing input file\n";
        return exitPrompt(-1, shouldPrompt);
    }

    std::string path = argv[1];
    bool isLoaded = loader.LoadBinaryFromFile(&model, &err, &warn, path); 
    if (!isLoaded) {
        std::cout << "Error: " << err << '\n';
        return exitPrompt(-1, shouldPrompt);
    }
    std::cout << "Loaded file \"" << path << "\"\n";

    std::string outPath;
    if (argc >= 3)
        outPath = argv[2];
    else {
        outPath = path.substr(0, path.size() - 4);
        outPath += ".jmd";
    }

    std::ofstream file;
    file.open(outPath, std::ios::out | std::ios::binary);
    if (!file.is_open()) {
        std::cout << "Error: Failed to create new JMD file\n";
        return exitPrompt(-1, shouldPrompt);
    }

    typedef std::pair<Mesh, mat4> MeshAndMatrix;
    std::vector<MeshAndMatrix> meshes;
    for (Node node : model.nodes) {
        int meshIndex = node.mesh;

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
    modelHeader.numMeshes = meshes.size();
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
            file.write((const char*)&vertex, sizeof(JStaticVertex));
        }

        // Write the indices buffer of the mesh
        std::cout << "\t\tWriting indices\n";
        file.write((const char*)indicesBuf.data, sizeof(uint16_t) * indicesBuf.count);
    }
    file.close();
    std::cout << "Finished compiling to file \"" << outPath << "\"\n";

    return exitPrompt(0, shouldPrompt);
}
