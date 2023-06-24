#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <tiny_gltf.h>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
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

    std::vector<Mesh> meshes;
    for (Node node : model.nodes) {
        int meshIndex = node.mesh;
        meshes.push_back(model.meshes[node.mesh]);
    }

    // Write the model header
    JModelHeader modelHeader;
    modelHeader.numMeshes = meshes.size();
    file.write((const char*)&modelHeader, sizeof(JModelHeader));
    std::cout << "Compiling model with " << modelHeader.numMeshes << " meshes\n";

    for (Mesh mesh : meshes) {
        // Get the meshes buffers
        JBuffer positionBuf = GetBuffer(model, mesh, "POSITION");
        JBuffer indicesBuf = GetIndices(model, mesh);

        // Write the mesh header
        JMeshHeader meshHeader;
        meshHeader.numVertices = positionBuf.count;
        meshHeader.numIndices = indicesBuf.count;
        file.write((const char*)&meshHeader, sizeof(JMeshHeader));
        std::cout << 
            "\tCompiling mesh with " << 
            meshHeader.numVertices << 
            " vertices and " << 
            meshHeader.numIndices << 
            " indices\n"; 

        // Write each vertex of the mesh one-by-one
        std::cout << "\t\tWriting vertices\n";
        for (int i = 0; i < positionBuf.count; i++) {
            JStaticVertex vertex;
            vertex.position = ((vec3*)positionBuf.data)[i];
            vertex.normal = vec3(0.0f);
            vertex.tangent = vec3(0.0f);
            vertex.bitangent = vec3(0.0f);
            vertex.uv = vec2(0.0f);

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
