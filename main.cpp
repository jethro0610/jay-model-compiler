#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <tiny_gltf.h>
#include <glm/glm.hpp>
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

int main() {
    Model model;
    TinyGLTF loader;
    std::string err;
    std::string warn;

    std::string path = "st_test.glb";

    bool isLoaded = loader.LoadBinaryFromFile(&model, &err, &warn, path); 
    if (!isLoaded) {
        std::cout << err;
        return -1;
    }
    std::cout << "Loaded file \"" << path << "\"\n";

    std::ofstream file;
    file.open("st_test.jmd", std::ios::out | std::ios::binary);
    if (!file.is_open()) {
        std::cout << "Failed to create new JMD file\n";
        return -1;
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
        JBuffer indices = GetIndices(model, mesh);

        // Write the mesh header
        JMeshHeader meshHeader;
        meshHeader.numVertices = positionBuf.count;
        meshHeader.numIndices = indices.count;
        file.write((const char*)&meshHeader, sizeof(JMeshHeader));
        std::cout << 
            "Compiling mesh with " << 
            meshHeader.numVertices << 
            " vertices and " << 
            meshHeader.numIndices << 
            " indices\n"; 

        // Write each vertex of the mesh one-by-one
        std::cout << "Writing vertices\n";
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
        std::cout << "Writing indices\n";
        file.write((const char*)&indices.data, sizeof(uint16_t) * indices.count);
    }

    return 0;
}
