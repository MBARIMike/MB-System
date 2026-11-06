#include "io/draco_mesh.h"

#ifdef MBMESH_WITH_DRACO

#include "draco/compression/decode.h"
#include "draco/compression/encode.h"

bool draco_available() {
    return true;
}

bool draco_encode_mesh(const std::vector<float> &positions, const std::vector<float> *normals, const std::vector<std::uint32_t> &indices,
                       int position_bits, int normal_bits, DracoMesh *result, std::string *error) {
    const auto fail = [&](const std::string &message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };
    if (result == nullptr || positions.empty() || positions.size() % 3 != 0 || indices.empty() || indices.size() % 3 != 0) {
        return fail("Draco: no mesh to encode");
    }
    if (position_bits < 2 || position_bits > 30 || normal_bits < 2 || normal_bits > 30) {
        return fail("Draco: the quantization bits must be 2 to 30");
    }
    const std::size_t point_count = positions.size() / 3;
    if (normals != nullptr && normals->size() != positions.size()) {
        return fail("Draco: the normals and the positions differ in number");
    }

    draco::Mesh mesh;
    mesh.set_num_points(static_cast<std::uint32_t>(point_count));
    draco::GeometryAttribute position_attribute;
    position_attribute.Init(draco::GeometryAttribute::POSITION, nullptr, 3, draco::DT_FLOAT32, false, 12, 0);
    const int position_attribute_id = mesh.AddAttribute(position_attribute, true, static_cast<std::uint32_t>(point_count));
    if (position_attribute_id < 0) {
        return fail("Draco: the position attribute could not be added");
    }
    int normal_attribute_id = -1;
    if (normals != nullptr) {
        draco::GeometryAttribute normal_attribute;
        normal_attribute.Init(draco::GeometryAttribute::NORMAL, nullptr, 3, draco::DT_FLOAT32, false, 12, 0);
        normal_attribute_id = mesh.AddAttribute(normal_attribute, true, static_cast<std::uint32_t>(point_count));
        if (normal_attribute_id < 0) {
            return fail("Draco: the normal attribute could not be added");
        }
    }
    for (std::size_t n = 0; n < point_count; n++) {
        mesh.attribute(position_attribute_id)->SetAttributeValue(draco::AttributeValueIndex(static_cast<std::uint32_t>(n)), &positions[3 * n]);
        if (normal_attribute_id >= 0) {
            mesh.attribute(normal_attribute_id)->SetAttributeValue(draco::AttributeValueIndex(static_cast<std::uint32_t>(n)), &(*normals)[3 * n]);
        }
    }
    for (std::size_t t = 0; t + 2 < indices.size(); t += 3) {
        mesh.SetFace(draco::FaceIndex(static_cast<std::uint32_t>(t / 3)),
                     {draco::PointIndex(indices[t]), draco::PointIndex(indices[t + 1]), draco::PointIndex(indices[t + 2])});
    }

    draco::Encoder encoder;
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, position_bits);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL, normal_bits);
    draco::EncoderBuffer buffer;
    const draco::Status status = encoder.EncodeMeshToBuffer(mesh, &buffer);
    if (!status.ok()) {
        return fail(std::string("Draco: encoding failed: ") + status.error_msg());
    }

    // Decoded again, to check it and to count what it holds.
    draco::DecoderBuffer decoder_buffer;
    decoder_buffer.Init(buffer.data(), buffer.size());
    draco::Decoder decoder;
    auto decoded = decoder.DecodeMeshFromBuffer(&decoder_buffer);
    if (!decoded.ok()) {
        return fail(std::string("Draco: the encoded mesh does not decode: ") + decoded.status().error_msg());
    }
    const std::unique_ptr<draco::Mesh> check = std::move(decoded).value();
    const draco::PointAttribute *decoded_position = check->GetNamedAttribute(draco::GeometryAttribute::POSITION);
    if (decoded_position == nullptr) {
        return fail("Draco: the encoded mesh has no positions");
    }
    result->data.assign(buffer.data(), buffer.data() + buffer.size());
    result->position_id = decoded_position->unique_id();
    const draco::PointAttribute *decoded_normal = check->GetNamedAttribute(draco::GeometryAttribute::NORMAL);
    result->normal_id = decoded_normal != nullptr ? static_cast<int>(decoded_normal->unique_id()) : -1;
    result->point_count = check->num_points();
    result->face_count = check->num_faces();
    return true;
}

bool draco_decode_mesh(const std::vector<unsigned char> &data, std::vector<float> *positions, std::vector<std::uint32_t> *indices, std::string *error) {
    draco::DecoderBuffer buffer;
    buffer.Init(reinterpret_cast<const char *>(data.data()), data.size());
    draco::Decoder decoder;
    auto decoded = decoder.DecodeMeshFromBuffer(&buffer);
    if (!decoded.ok()) {
        if (error != nullptr) {
            *error = decoded.status().error_msg();
        }
        return false;
    }
    const std::unique_ptr<draco::Mesh> mesh = std::move(decoded).value();
    const draco::PointAttribute *attribute = mesh->GetNamedAttribute(draco::GeometryAttribute::POSITION);
    if (attribute == nullptr) {
        return false;
    }
    positions->clear();
    for (draco::PointIndex point(0); point < mesh->num_points(); point++) {
        float value[3];
        attribute->GetMappedValue(point, value);
        positions->insert(positions->end(), value, value + 3);
    }
    indices->clear();
    for (draco::FaceIndex face(0); face < mesh->num_faces(); face++) {
        for (int k = 0; k < 3; k++) {
            indices->push_back(mesh->face(face)[k].value());
        }
    }
    return true;
}

#else

bool draco_available() {
    return false;
}

bool draco_encode_mesh(const std::vector<float> &, const std::vector<float> *, const std::vector<std::uint32_t> &, int, int, DracoMesh *, std::string *error) {
    if (error != nullptr) {
        *error = "this build of mbmesh does not include Draco";
    }
    return false;
}

bool draco_decode_mesh(const std::vector<unsigned char> &, std::vector<float> *, std::vector<std::uint32_t> *, std::string *error) {
    if (error != nullptr) {
        *error = "this build of mbmesh does not include Draco";
    }
    return false;
}

#endif
