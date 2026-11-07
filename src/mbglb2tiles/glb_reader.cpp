#include "glb_reader.h"

#include "draco_mesh.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#define TINYGLTF_NO_STB_IMAGE
#define TINYGLTF_NO_STB_IMAGE_WRITE
#include "tinygltf/tiny_gltf.h"

namespace {

void set_error(std::string *error, const std::string &message) {
    if (error != nullptr) {
        *error = message;
    }
}

// The bytes of an accessor: where element 0 is, and the distance between elements.
[[nodiscard]] bool accessor_bytes(const tinygltf::Model &model, const tinygltf::Accessor &accessor, const unsigned char **start, std::size_t *stride, std::string *error) {
    if (accessor.bufferView < 0 || static_cast<std::size_t>(accessor.bufferView) >= model.bufferViews.size()) {
        set_error(error, "an accessor has no buffer view");
        return false;
    }
    const tinygltf::BufferView &view = model.bufferViews[static_cast<std::size_t>(accessor.bufferView)];
    if (view.buffer < 0 || static_cast<std::size_t>(view.buffer) >= model.buffers.size()) {
        set_error(error, "a buffer view has no buffer");
        return false;
    }
    const int byte_stride = accessor.ByteStride(view);
    if (byte_stride <= 0) {
        set_error(error, "an accessor has an invalid stride");
        return false;
    }
    const std::vector<unsigned char> &data = model.buffers[static_cast<std::size_t>(view.buffer)].data;
    const std::size_t first = view.byteOffset + accessor.byteOffset;
    if (accessor.count > 0 && first + static_cast<std::size_t>(byte_stride) * (accessor.count - 1) + 1 > data.size()) {
        set_error(error, "an accessor reaches beyond its buffer");
        return false;
    }
    *start = data.data() + first;
    *stride = static_cast<std::size_t>(byte_stride);
    return true;
}

[[nodiscard]] bool node_is_identity(const tinygltf::Node &node) {
    const auto near = [](double a, double b) { return std::fabs(a - b) < 1.0e-12; };
    if (!node.matrix.empty()) {
        for (std::size_t k = 0; k < node.matrix.size(); k++) {
            if (!near(node.matrix[k], (k % 5 == 0) ? 1.0 : 0.0)) {
                return false;
            }
        }
    }
    for (std::size_t k = 0; k < node.translation.size(); k++) {
        if (!near(node.translation[k], 0.0)) {
            return false;
        }
    }
    for (std::size_t k = 0; k < node.rotation.size(); k++) {
        if (!near(node.rotation[k], k == 3 ? 1.0 : 0.0)) {
            return false;
        }
    }
    for (std::size_t k = 0; k < node.scale.size(); k++) {
        if (!near(node.scale[k], 1.0)) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] double number_of(const tinygltf::Value &value) {
    return value.IsNumber() ? value.GetNumberAsDouble() : 0.0;
}

} // namespace

bool read_mesh_glb(const std::filesystem::path &path, GlbMesh *result, std::string *error) {
    *result = GlbMesh();
    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string load_error, warning;
    if (!loader.LoadBinaryFromFile(&model, &load_error, &warning, path.string())) {
        set_error(error, "cannot read " + path.string() + ": " + load_error);
        return false;
    }
    result->generator = model.asset.generator;

    // Where the vertices are: the georeference block that mbmesh and mbgrd2gltf write.
    const tinygltf::Value &extras = model.asset.extras;
    if (!extras.IsObject() || !extras.Has("MB-System") || !extras.Has("georeference")) {
        set_error(error, path.string() + " has no georeference in its extras, so it was not written by mbmesh or mbgrd2gltf, and where it is on the Earth is not known");
        return false;
    }
    const tinygltf::Value &georeference = extras.Get("georeference");
    const std::string convention = georeference.Has("vertexCoordinates") ? georeference.Get("vertexCoordinates").Get<std::string>() : "";
    if (convention == "relativeToCenter") {
        const tinygltf::Value &center = georeference.Get("center");
        if (!center.IsArray() || center.ArrayLen() != 3) {
            set_error(error, "the georeference has vertexCoordinates relativeToCenter, without a center of three numbers");
            return false;
        }
        result->has_center = true;
        result->center = Vec3(number_of(center.Get(0)), number_of(center.Get(1)), number_of(center.Get(2)));
    } else if (convention != "absolute") {
        set_error(error, "the georeference has vertexCoordinates '" + convention + "', not absolute or relativeToCenter");
        return false;
    }
    const tinygltf::Value &content = extras.Get("MB-System").Get("content");
    if (content.IsObject()) {
        if (content.Has("compression") && content.Get("compression").Has("positionStep")) {
            result->position_step = number_of(content.Get("compression").Get("positionStep"));
        }
        if (content.Has("description") && content.Get("description").IsString()) {
            result->description = content.Get("description").Get<std::string>();
        }
    }

    for (const tinygltf::Node &node : model.nodes) {
        if (node.mesh >= 0 && !node_is_identity(node)) {
            set_error(error, "a node of the file has a transform, which is not handled: the vertices of mbmesh and mbgrd2gltf are in the file's frame");
            return false;
        }
    }

    Mesh &mesh = result->mesh;
    for (const tinygltf::Mesh &gltf_mesh : model.meshes) {
        for (const tinygltf::Primitive &primitive : gltf_mesh.primitives) {
            if (primitive.mode != TINYGLTF_MODE_TRIANGLES) {
                set_error(error, "the file has a primitive that is not triangles");
                return false;
            }
            const std::size_t base = mesh.vertices.size();
            const auto draco = primitive.extensions.find("KHR_draco_mesh_compression");
            if (draco != primitive.extensions.end()) {
                const int view_index = draco->second.Get("bufferView").GetNumberAsInt();
                if (view_index < 0 || static_cast<std::size_t>(view_index) >= model.bufferViews.size()) {
                    set_error(error, "a Draco primitive names a buffer view that is not there");
                    return false;
                }
                const tinygltf::BufferView &view = model.bufferViews[static_cast<std::size_t>(view_index)];
                const unsigned char *start = model.buffers.at(static_cast<std::size_t>(view.buffer)).data.data() + view.byteOffset;
                std::vector<float> positions;
                std::vector<std::uint32_t> indices;
                std::string decode_error;
                if (!mbdraco::decode(std::vector<unsigned char>(start, start + view.byteLength), &positions, &indices, &decode_error)) {
                    set_error(error, "cannot decode a Draco primitive: " + decode_error);
                    return false;
                }
                for (std::size_t k = 0; k + 2 < positions.size(); k += 3) {
                    mesh.vertices.push_back(Vec3(positions[k], positions[k + 1], positions[k + 2]));
                }
                for (const std::uint32_t index : indices) {
                    mesh.indices.push_back(static_cast<unsigned int>(base + index));
                }
                result->draco_primitives++;
            } else {
                const auto found = primitive.attributes.find("POSITION");
                if (found == primitive.attributes.end() || primitive.indices < 0) {
                    set_error(error, "a primitive has no POSITION or no indices");
                    return false;
                }
                const tinygltf::Accessor &position = model.accessors.at(static_cast<std::size_t>(found->second));
                if (position.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT || position.type != TINYGLTF_TYPE_VEC3) {
                    set_error(error, "POSITION is not three floats");
                    return false;
                }
                const unsigned char *start = nullptr;
                std::size_t stride = 0;
                if (!accessor_bytes(model, position, &start, &stride, error)) {
                    return false;
                }
                for (std::size_t k = 0; k < position.count; k++) {
                    float value[3];
                    std::memcpy(value, start + k * stride, sizeof(value));
                    mesh.vertices.push_back(Vec3(value[0], value[1], value[2]));
                }
                const tinygltf::Accessor &index = model.accessors.at(static_cast<std::size_t>(primitive.indices));
                if (!accessor_bytes(model, index, &start, &stride, error)) {
                    return false;
                }
                for (std::size_t k = 0; k < index.count; k++) {
                    std::uint32_t value = 0;
                    switch (index.componentType) {
                    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                        value = start[k * stride];
                        break;
                    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
                        std::uint16_t small;
                        std::memcpy(&small, start + k * stride, sizeof(small));
                        value = small;
                        break;
                    }
                    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
                        std::memcpy(&value, start + k * stride, sizeof(value));
                        break;
                    default:
                        set_error(error, "the indices are not unsigned integers");
                        return false;
                    }
                    mesh.indices.push_back(static_cast<unsigned int>(base + value));
                }
            }
            result->primitives++;
        }
    }
    if (mesh.vertices.empty() || mesh.indices.empty() || mesh.indices.size() % 3 != 0) {
        set_error(error, path.string() + " holds no triangles");
        return false;
    }
    for (const unsigned int index : mesh.indices) {
        if (index >= mesh.vertices.size()) {
            set_error(error, path.string() + " has an index beyond its vertices");
            return false;
        }
    }
    return true;
}
