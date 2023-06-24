#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <tinygltf/tiny_gltf.h>
#include <iostream>
#include <string>

using namespace tinygltf;

int main() {
    Model model;
    TinyGLTF loader;
    std::string err;
    std::string warn;

    std::string path = "./st_test.gltf";

    bool isLoaded = loader.LoadASCIIFromFile(&model, &err, &warn, path); 
    if (!isLoaded) {
        std::cout << "Failed to load file \"" << path << "\"\n";
        return -1;
    }
    std::cout << "Loaded file "<< "\"\n";

    return 0;
}
