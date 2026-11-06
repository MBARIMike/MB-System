#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// Draco compression of a triangle mesh, for the KHR_draco_mesh_compression extension of glTF. The Draco encoder is the source
// that mbgrd2gltf holds (src/mbgrd2gltf/draco); a build without it (MBMESH_WITH_DRACO not defined, as the autotools build is)
// has draco_available() false and writes GLBs without compression.
struct DracoMesh {
    std::vector<unsigned char> data;   // the compressed mesh, to be the content of a buffer view
    int position_id = 0;               // the unique ids of the attributes in the compressed mesh, as the extension names them
    int normal_id = -1;                // -1 for a mesh without normals
    std::size_t point_count = 0;       // what the compressed mesh decodes to, which can differ from what was encoded
    std::size_t face_count = 0;        // (a vertex that touches the surface twice is made two, and a degenerate triangle goes)
};

[[nodiscard]] bool draco_available();

// Encodes the mesh: positions (x, y, z for each vertex), normals (the same, or null), and three indices for each triangle, with
// the positions in position_bits bits (2 to 30) over the largest of the ranges of the three axes, and the normals in
// normal_bits bits each. The result is decoded again to check it, and to count what it holds. Returns false and says why in
// *error if it cannot.
[[nodiscard]] bool draco_encode_mesh(const std::vector<float> &positions, const std::vector<float> *normals, const std::vector<std::uint32_t> &indices,
                                     int position_bits, int normal_bits, DracoMesh *result, std::string *error);

// The positions and faces of a compressed mesh, for tests: three floats for each point, three indices for each face.
[[nodiscard]] bool draco_decode_mesh(const std::vector<unsigned char> &data, std::vector<float> *positions, std::vector<std::uint32_t> *indices, std::string *error);
