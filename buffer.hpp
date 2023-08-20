#pragma once
#include <tiny_gltf.h>

template <typename T>
class Buffer {
    T* data_; 
    int size_;

public:
    Buffer(tinygltf::Model& model, tinygltf::Accessor& accessor) {
        tinygltf::BufferView& view = model.bufferViews[accessor.bufferView];
        tinygltf::Buffer& buffer = model.buffers[view.buffer];

        int dataOffset = view.byteOffset + accessor.byteOffset;
        data_ = (T*)&buffer.data[dataOffset];
        size_ = accessor.count;
    }

    Buffer(tinygltf::Model& model, tinygltf::Mesh& mesh, std::string attributeName) {
        tinygltf::Accessor accessor;
        if (attributeName == "INDICES")
            accessor = model.accessors[mesh.primitives[0].indices];
        else
            accessor = model.accessors[mesh.primitives[0].attributes.at(attributeName)];

        tinygltf::BufferView& view = model.bufferViews[accessor.bufferView];
        tinygltf::Buffer& buffer = model.buffers[view.buffer];

        int dataOffset = view.byteOffset + accessor.byteOffset;
        data_ = (T*)&buffer.data[dataOffset];
        size_ = accessor.count;
    };

    T& operator[](int index) const {
        assert(index < size_);
        return data_[index];
    }

    const int size() const {
        return size_;
    }

    T* data() const {
        return data_;
    }
};
