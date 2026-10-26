#include "io/tileset_writer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>

namespace {

void set_error(std::string *error, const std::string &message) {
    if (error != nullptr) {
        *error = message;
    }
}

std::string number_text(double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.17g", value);
    return buffer;
}

std::string json_string(const std::string &text) {
    std::string result = "\"";
    for (const char character : text) {
        switch (character) {
        case '"':
            result += "\\\"";
            break;
        case '\\':
            result += "\\\\";
            break;
        case '\n':
            result += "\\n";
            break;
        case '\t':
            result += "\\t";
            break;
        default:
            if (static_cast<unsigned char>(character) < 0x20) {
                char buffer[8];
                std::snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned>(character));
                result += buffer;
            } else {
                result += character;
            }
        }
    }
    return result + "\"";
}

void write_meta_value(std::ostream &out, const MetaValue &value, int depth) {
    const std::string indent(static_cast<std::size_t>(depth) * 2, ' ');
    const std::string inner(static_cast<std::size_t>(depth + 1) * 2, ' ');
    switch (value.type) {
    case MetaValue::Type::String:
        out << json_string(value.text);
        break;
    case MetaValue::Type::Number:
        out << metadata_number_text(value.number);
        break;
    case MetaValue::Type::Bool:
        out << (value.flag ? "true" : "false");
        break;
    case MetaValue::Type::Array:
        out << "[";
        for (std::size_t i = 0; i < value.items.size(); i++) {
            out << (i == 0 ? "" : ", ");
            write_meta_value(out, value.items[i], depth + 1);
        }
        out << "]";
        break;
    case MetaValue::Type::Object:
        out << "{\n";
        for (std::size_t i = 0; i < value.members.size(); i++) {
            out << inner << json_string(value.members[i].first) << ": ";
            write_meta_value(out, value.members[i].second, depth + 1);
            out << (i + 1 < value.members.size() ? ",\n" : "\n");
        }
        out << indent << "}";
        break;
    }
}

// A 3D Tiles box: the center and the three half axes, here along the axes of the frame.
std::string box_text(const double minimum[3], const double maximum[3]) {
    std::string text = "[";
    for (int i = 0; i < 3; i++) {
        text += number_text((minimum[i] + maximum[i]) / 2.0) + ", ";
    }
    for (int axis = 0; axis < 3; axis++) {
        for (int i = 0; i < 3; i++) {
            text += number_text(i == axis ? (maximum[axis] - minimum[axis]) / 2.0 : 0.0);
            text += (axis == 2 && i == 2) ? "" : ", ";
        }
    }
    return text + "]";
}

} // namespace

Mesh tileset_content_mesh(const Mesh &local_mesh) {
    Mesh result;
    result.vertices.reserve(local_mesh.vertices.size());
    for (const Vec3 &vertex : local_mesh.vertices) {
        result.vertices.push_back(Vec3(vertex.x, vertex.z, -vertex.y));
    }
    result.normals.reserve(local_mesh.normals.size());
    for (const Vec3 &normal : local_mesh.normals) {
        result.normals.push_back(Vec3(normal.x, normal.z, -normal.y));
    }
    result.indices = local_mesh.indices;
    return result;
}

bool mesh_bounds(const Mesh &mesh, double minimum[3], double maximum[3]) {
    if (mesh.vertices.empty()) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        minimum[i] = std::numeric_limits<double>::max();
        maximum[i] = std::numeric_limits<double>::lowest();
    }
    for (const Vec3 &vertex : mesh.vertices) {
        const double values[3] = {vertex.x, vertex.y, vertex.z};
        for (int i = 0; i < 3; i++) {
            minimum[i] = std::min(minimum[i], values[i]);
            maximum[i] = std::max(maximum[i], values[i]);
        }
    }
    return true;
}

std::vector<double> tileset_transform(const CoordinateFrame &frame) {
    return {frame.east.x,  frame.east.y,  frame.east.z,  0.0,
            frame.north.x, frame.north.y, frame.north.z, 0.0,
            frame.up.x,    frame.up.y,    frame.up.z,    0.0,
            frame.origin_ecef.x, frame.origin_ecef.y, frame.origin_ecef.z, 1.0};
}

bool write_tileset_json(
    const std::filesystem::path &path,
    const CoordinateFrame &frame,
    const std::vector<TilesetTile> &tiles,
    const MetaValue &extras,
    const std::string &generator,
    std::string *error) {

    if (tiles.empty()) {
        set_error(error, "no tiles to write a tileset for");
        return false;
    }

    double minimum[3], maximum[3];
    for (int i = 0; i < 3; i++) {
        minimum[i] = std::numeric_limits<double>::max();
        maximum[i] = std::numeric_limits<double>::lowest();
    }
    for (const TilesetTile &tile : tiles) {
        for (int i = 0; i < 3; i++) {
            minimum[i] = std::min(minimum[i], tile.minimum[i]);
            maximum[i] = std::max(maximum[i], tile.maximum[i]);
        }
    }
    const double diagonal = std::sqrt((maximum[0] - minimum[0]) * (maximum[0] - minimum[0]) + (maximum[1] - minimum[1]) * (maximum[1] - minimum[1]) +
                                      (maximum[2] - minimum[2]) * (maximum[2] - minimum[2]));

    std::ofstream out(path);
    if (!out) {
        set_error(error, "Unable to create tileset file: " + path.string());
        return false;
    }

    const std::vector<double> transform = tileset_transform(frame);
    out << "{\n"
        << "  \"asset\": {\"version\": \"1.1\", \"generator\": " << json_string(generator) << "},\n"
        << "  \"geometricError\": " << number_text(diagonal) << ",\n";
    if (!extras.members.empty()) {
        out << "  \"extras\": ";
        write_meta_value(out, extras, 1);
        out << ",\n";
    }
    out << "  \"root\": {\n"
        << "    \"transform\": [";
    for (std::size_t i = 0; i < transform.size(); i++) {
        out << (i == 0 ? "" : ", ") << number_text(transform[i]);
    }
    out << "],\n"
        << "    \"boundingVolume\": {\"box\": " << box_text(minimum, maximum) << "},\n"
        << "    \"geometricError\": " << number_text(diagonal) << ",\n"
        << "    \"refine\": \"ADD\",\n"
        << "    \"children\": [\n";
    for (std::size_t i = 0; i < tiles.size(); i++) {
        out << "      {\"boundingVolume\": {\"box\": " << box_text(tiles[i].minimum, tiles[i].maximum) << "}, \"geometricError\": 0, \"content\": {\"uri\": "
            << json_string(tiles[i].uri) << "}}" << (i + 1 < tiles.size() ? ",\n" : "\n");
    }
    out << "    ]\n"
        << "  }\n"
        << "}\n";

    out.close();
    if (!out) {
        set_error(error, "Unable to write tileset file: " + path.string());
        return false;
    }
    return true;
}
