#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "../data_types/geometry.h"
#include "../data_types/swathfile.h"
#include "provenance.h"

/*
 * OGC 3D Tiles output: the tiles of a chunked reconstruction as a tileset that a viewer can load a part at
 * a time, instead of one file of the whole surface.
 *
 * A tile's content is a GLB in the local east-north-up frame of the reconstruction, written Y-up as glTF
 * requires and as 3D Tiles expects (X east, Y up, Z south). Loaders rotate it to Z-up (the frame of the
 * tileset, east-north-up), and the transform of the root tile, which is the east-north-up frame at the
 * frame origin expressed in WGS 84 ECEF, places the tileset on the Earth. Bounding volumes are boxes in
 * the east-north-up frame, which is the tileset's coordinate system before the transform.
 */

// One tile of the tileset: its content and where it is, in the local east-north-up frame (X east, Y north, Z up).
struct TilesetTile {
    std::string uri;           // relative to tileset.json
    double minimum[3] = {0.0, 0.0, 0.0};
    double maximum[3] = {0.0, 0.0, 0.0};
};

// The mesh in the form tileset content has: local east-north-up (x east, y north, z up) to glTF Y-up
// (x east, y up, z south), a rotation, so the winding is kept.
[[nodiscard]] Mesh tileset_content_mesh(const Mesh &local_mesh);

// The bounds of a mesh in local east-north-up coordinates. Returns false for an empty mesh.
[[nodiscard]] bool mesh_bounds(const Mesh &mesh, double minimum[3], double maximum[3]);

// The east-north-up frame at the origin of the frame, in WGS 84 ECEF, as the 16 numbers of a 3D Tiles
// transform: a 4x4 matrix in column-major order, whose columns are east, north, up and the origin.
[[nodiscard]] std::vector<double> tileset_transform(const CoordinateFrame &frame);

// Writes tileset.json: a root tile with the transform and no content, whose children are the tiles, each a
// leaf. The geometric error of the root is the diagonal of its bounding box, so a viewer always refines to
// the tiles, and that of each tile is 0. If extras is not empty it is written as the tileset's extras.
[[nodiscard]] bool write_tileset_json(
    const std::filesystem::path &path,
    const CoordinateFrame &frame,
    const std::vector<TilesetTile> &tiles,
    const MetaValue &extras,
    const std::string &generator,
    std::string *error);
