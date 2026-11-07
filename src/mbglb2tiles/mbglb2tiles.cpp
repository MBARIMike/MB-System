/*
 * mbglb2tiles: OGC 3D Tiles of levels of detail from a GLB mesh of mbmesh or mbgrd2gltf.
 *
 * The mesh is read, welded (the primitives of a GLB each have their own copies of the vertices on their edges), put in the local east-north-up
 * frame at the middle of its bounds, and made coarser level by level by clustering vertices on a lattice that doubles in size at each level. Each
 * level is cut into the tiles of a lattice of squares that also doubles, so that four tiles of a level are replaced by one of the next. The tiles
 * are GLBs (Y-up, Draco-compressed unless asked not to) and tileset.json is the tree of them, with the transform that puts the frame on the Earth.
 */

#include "glb_reader.h"
#include "mesh_levels.h"

#include "algorithms/seam_blending.h"
#include "io/coordinate_frame.h"
#include "io/glb_writer.h"
#include "io/provenance.h"
#include "io/tileset_writer.h"
#include "util/logger.h"
#include "util/memory.h"
#include "util/parallel.h"
#include "util/stopwatch.h"

#include "draco_mesh.h"
#include "mb_define.h"

#include <getopt.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <thread>
#include <filesystem>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace {

// The tile of the finest level is this many times the spacing of the mesh, as the block of mbgrd2gltf is 256 cells. The lattice of the clustering of a
// level is the spacing times two to the power of the level, and so is the tile, so a tile has about as many vertices at every level.
constexpr double cells_per_tile = 256.0;

struct Options {
    std::filesystem::path input;
    std::filesystem::path output;
    double tile_size = 0.0;          // 0: chosen from the spacing of the mesh
    double weld_tolerance = 0.0;     // 0: chosen from the quantization the file says
    int threads = 0;                 // 0: the cores, at most 8
    mbdraco::Settings draco;
    bool verbose = false;
};

struct TileRecord {
    std::string uri;
    double minimum[3];
    double maximum[3];
};

void print_usage() {
    std::cout <<
        "mbglb2tiles: an OGC 3D Tiles tileset of levels of detail from a GLB mesh made by mbmesh or mbgrd2gltf\n\n"
        "usage: mbglb2tiles -I mesh.glb [-O outputdir] [options]\n\n"
        "  -I, --input <file.glb>       The mesh: a GLB of mbmesh or mbgrd2gltf (with the georeference in its extras)\n"
        "  -O, --output <outputdir>     Where tileset.json and the tiles go [mbglb2tiles_output beside the input]\n"
        "      --tile-size <meters>     The side of the tiles of the finest level [256 times the spacing of the mesh, to two digits]; each level\n"
        "                               above has tiles of twice the side, until one tile holds everything\n"
        "      --weld-tolerance <m>     Vertices of the file this close are one [twice the quantization step the file records, or 0.001]\n"
        "      --threads <n>            Tiles written at once [the cores, at most 8]\n"
        "      --no-draco               Write the tiles as arrays [compressed with Draco]\n"
        "      --draco-precision <m>    With Draco, keep positions to this many meters [0.001]\n"
        "      --draco-normal-bits <n>  With Draco, the bits of each normal, 2-30 [10]\n"
        "  -V, --verbose                Log the stages of every level\n"
        "  -h, --help                   This text\n";
}

[[nodiscard]] double round_to_two_digits(double value) {
    const double scale = std::pow(10.0, std::floor(std::log10(value)) - 1.0);
    return std::round(value / scale) * scale;
}

[[nodiscard]] bool parse_positive(const char *text, double *value) {
    char *end = nullptr;
    *value = std::strtod(text, &end);
    return end != text && *end == '\0' && std::isfinite(*value) && *value > 0.0;
}

} // namespace

int main(int argc, char **argv) {
    mbmesh::Logger::set_level(mbmesh::LogLevel::INFO);
    Options options;
    static const struct option long_options[] = {
        {"input", required_argument, nullptr, 'I'},
        {"output", required_argument, nullptr, 'O'},
        {"tile-size", required_argument, nullptr, 1000},
        {"weld-tolerance", required_argument, nullptr, 1001},
        {"threads", required_argument, nullptr, 1002},
        {"no-draco", no_argument, nullptr, 1003},
        {"draco-precision", required_argument, nullptr, 1004},
        {"draco-normal-bits", required_argument, nullptr, 1005},
        {"verbose", no_argument, nullptr, 'V'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0}};
    int option;
    while ((option = getopt_long(argc, argv, "I:O:Vh", long_options, nullptr)) != -1) {
        std::string problem;
        switch (option) {
        case 'I':
            options.input = optarg;
            break;
        case 'O':
            options.output = optarg;
            break;
        case 1000:
            if (!parse_positive(optarg, &options.tile_size)) {
                LOG_ERROR("invalid --tile-size:", std::string("'") + optarg + "' is not a size in meters, more than zero");
                return 1;
            }
            break;
        case 1001:
            if (!parse_positive(optarg, &options.weld_tolerance)) {
                LOG_ERROR("invalid --weld-tolerance:", std::string("'") + optarg + "' is not a size in meters, more than zero");
                return 1;
            }
            break;
        case 1002:
            options.threads = std::max(1, std::atoi(optarg));
            break;
        case 1003:
            options.draco.enabled = false;
            break;
        case 1004:
            if (!mbdraco::parse_precision(optarg, &options.draco.precision, &problem)) {
                LOG_ERROR("invalid --draco-precision:", problem);
                return 1;
            }
            break;
        case 1005:
            if (!mbdraco::parse_normal_bits(optarg, &options.draco.normal_bits, &problem)) {
                LOG_ERROR("invalid --draco-normal-bits:", problem);
                return 1;
            }
            break;
        case 'V':
            options.verbose = true;
            mbmesh::Logger::set_level(mbmesh::LogLevel::DEBUG);
            break;
        case 'h':
            print_usage();
            return 0;
        default:
            print_usage();
            return 1;
        }
    }
    if (options.input.empty()) {
        LOG_ERROR("no input: give a GLB with -I");
        print_usage();
        return 1;
    }
    if (options.output.empty()) {
        options.output = options.input.parent_path() / "mbglb2tiles_output";
    }
    if (options.threads <= 0) {
        options.threads = static_cast<int>(std::min(8u, std::max(1u, std::thread::hardware_concurrency())));
    }
    const mbmesh::Stopwatch run_timer;

    // ---- Read the mesh and put it in the local frame ----
    GlbMesh glb;
    std::string error;
    {
        const mbmesh::Stopwatch timer;
        if (!read_mesh_glb(options.input, &glb, &error)) {
            LOG_ERROR(error);
            return 1;
        }
        LOG_INFO("[input]", "read " + std::to_string(glb.mesh.vertices.size()) + " vertices and " + std::to_string(glb.mesh.indices.size() / 3) + " triangles from " +
                                std::to_string(glb.primitives) + " primitives (" + std::to_string(glb.draco_primitives) + " Draco) in " + timer.text());
    }
    if (!glb.has_center) {
        LOG_WARN("the vertices of the file are absolute ECEF stored as 32-bit floats, which resolve about 0.5 m: the tiles cannot be better than that; make the mesh with --geoorigin");
    }
    Mesh &mesh = glb.mesh;
    double ecef_minimum[3] = {1e300, 1e300, 1e300}, ecef_maximum[3] = {-1e300, -1e300, -1e300};
    for (Vec3 &vertex : mesh.vertices) {
        if (glb.has_center) {
            vertex = vertex + glb.center;
        }
        const double values[3] = {vertex.x, vertex.y, vertex.z};
        for (int k = 0; k < 3; k++) {
            ecef_minimum[k] = std::min(ecef_minimum[k], values[k]);
            ecef_maximum[k] = std::max(ecef_maximum[k], values[k]);
        }
    }
    const Vec3 middle(0.5 * (ecef_minimum[0] + ecef_maximum[0]), 0.5 * (ecef_minimum[1] + ecef_maximum[1]), 0.5 * (ecef_minimum[2] + ecef_maximum[2]));
    const CoordinateFrame frame = coordinate_frame_from_origin(ecef_to_geodetic(middle));
    for (Vec3 &vertex : mesh.vertices) {
        vertex = ecef_to_local(vertex, frame);
    }

    // ---- Weld the copies of the vertices on the edges of the primitives, and make the normals ----
    {
        const mbmesh::Stopwatch timer;
        const double tolerance = options.weld_tolerance > 0.0 ? options.weld_tolerance : (glb.position_step > 0.0 ? std::max(0.001, 2.0 * glb.position_step) : 0.001);
        const std::size_t before = mesh.vertices.size();
        weld_coincident_vertices(&mesh, tolerance);
        compute_vertex_normals(&mesh);
        LOG_INFO("[weld]", "vertices within " + std::to_string(tolerance) + " m joined: " + std::to_string(before) + " to " + std::to_string(mesh.vertices.size()) + " in " + timer.text());
    }

    // ---- The lattices ----
    double minimum[3] = {1e300, 1e300, 1e300}, maximum[3] = {-1e300, -1e300, -1e300};
    for (const Vec3 &vertex : mesh.vertices) {
        const double values[3] = {vertex.x, vertex.y, vertex.z};
        for (int k = 0; k < 3; k++) {
            minimum[k] = std::min(minimum[k], values[k]);
            maximum[k] = std::max(maximum[k], values[k]);
        }
    }
    const double spacing = native_spacing(mesh);
    if (!(spacing > 0.0)) {
        LOG_ERROR("the mesh has no area seen from above, so there is no spacing to choose tiles from");
        return 1;
    }
    const double tile_size = options.tile_size > 0.0 ? options.tile_size : round_to_two_digits(cells_per_tile * spacing);
    const double extent = std::max(maximum[0] - minimum[0], maximum[1] - minimum[1]);
    int top = 0;
    while (tile_size * std::pow(2.0, top) <= extent * (1.0 + 1e-9)) {
        top++;
    }
    const Vec3 anchor(minimum[0], minimum[1], minimum[2]);
    LOG_INFO("[levels]", "spacing " + std::to_string(spacing) + " m; tiles of " + std::to_string(tile_size) + " m at the finest level, " + std::to_string(top + 1) +
                             " levels, the last of one tile of " + std::to_string(tile_size * std::pow(2.0, top)) + " m, over " + std::to_string(extent) + " m");

    // The metadata of the tiles: the frame they are in, and where it is.
    RunInfo run;
    run.program = "mbglb2tiles";
    run.version = MB_VERSION;
    run.datalist = std::filesystem::absolute(options.input).string();
    run.geoorigin_mode = "none";
    run.has_center = true;
    run.local_frame = true;
    run.center_x = frame.origin_ecef.x;
    run.center_y = frame.origin_ecef.y;
    run.center_z = frame.origin_ecef.z;
    run.center_longitude = frame.origin.longitude;
    run.center_latitude = frame.origin.latitude;
    run.center_height = frame.origin.elevation;
    {
        double west = 1e300, south = 1e300, east = -1e300, north = -1e300;
        for (const double x : {minimum[0], maximum[0]}) {
            for (const double y : {minimum[1], maximum[1]}) {
                const GeodeticPosition corner = local_to_geodetic(Vec3(x, y, 0.5 * (minimum[2] + maximum[2])), frame);
                west = std::min(west, corner.longitude);
                east = std::max(east, corner.longitude);
                south = std::min(south, corner.latitude);
                north = std::max(north, corner.latitude);
            }
        }
        run.extent_west = west;
        run.extent_south = south;
        run.extent_east = east;
        run.extent_north = north;
        run.mean_elevation = frame.origin.elevation + 0.5 * (minimum[2] + maximum[2]);
    }
    const MetaValue metadata = run_metadata(run);
    set_glb_compression(options.draco);

    std::error_code directory_error;
    std::filesystem::create_directories(options.output, directory_error);
    if (directory_error) {
        LOG_ERROR("cannot create " + options.output.string() + ": " + directory_error.message());
        return 1;
    }

    // ---- Each level: coarsen, cut into tiles, write them ----
    std::vector<std::map<std::pair<int, int>, TileRecord>> records(static_cast<std::size_t>(top) + 1);
    Mesh coarse;
    std::size_t tiles_written = 0, bytes_written = 0;
    for (int level = 0; level <= top; level++) {
        const mbmesh::Stopwatch level_timer;
        const double size = tile_size * std::pow(2.0, level);
        const Mesh *level_mesh = &mesh;
        if (level > 0) {
            if (!cluster_mesh(mesh, anchor, spacing * std::pow(2.0, level), &coarse, &error)) {
                LOG_ERROR(error);
                return 1;
            }
            compute_vertex_normals(&coarse);
            level_mesh = &coarse;
        }
        const std::vector<MeshTile> tiles = cut_mesh(*level_mesh, minimum[0], minimum[1], size);
        const std::filesystem::path directory = options.output / ("L" + std::to_string(level));
        std::filesystem::create_directories(directory, directory_error);
        std::vector<TileRecord> made(tiles.size());
        std::atomic<bool> failed(false);
        std::mutex error_mutex;
        mbmesh::parallel_for_ranges(0, static_cast<long>(tiles.size()), options.threads, [&](long first, long last) {
            for (long n = first; n < last && !failed; n++) {
                const MeshTile &tile = tiles[static_cast<std::size_t>(n)];
                TileRecord &record = made[static_cast<std::size_t>(n)];
                record.uri = "L" + std::to_string(level) + "/tile_" + std::to_string(tile.i) + "_" + std::to_string(tile.j) + ".glb";
                if (!mesh_bounds(tile.mesh, record.minimum, record.maximum)) {
                    continue;
                }
                GlbContent content{"tileset_tile", "Level " + std::to_string(level) + " of " + std::to_string(top) + " of the tileset, in the east-north-up frame at the center, Y-up", true, tile.i, tile.j};
                content.metadata = &metadata;
                std::string tile_error;
                if (!write_mesh_glb_file(options.output / record.uri, tileset_content_mesh(tile.mesh), &tile_error, content)) {
                    std::lock_guard<std::mutex> lock(error_mutex);
                    LOG_ERROR(tile_error);
                    failed = true;
                }
            }
        });
        if (failed) {
            return 1;
        }
        std::size_t level_bytes = 0;
        for (std::size_t n = 0; n < tiles.size(); n++) {
            records[static_cast<std::size_t>(level)][{tiles[n].i, tiles[n].j}] = made[n];
            std::error_code size_error;
            const auto file_size = std::filesystem::file_size(options.output / made[n].uri, size_error);
            level_bytes += size_error ? 0 : static_cast<std::size_t>(file_size);
        }
        tiles_written += tiles.size();
        bytes_written += level_bytes;
        LOG_INFO("[level]", "level " + std::to_string(level) + ": " + std::to_string(level_mesh->vertices.size()) + " vertices, " + std::to_string(level_mesh->indices.size() / 3) +
                                " triangles, " + std::to_string(tiles.size()) + " tiles, " + std::to_string(level_bytes / 1024) + " KiB, in " + level_timer.text());
    }

    // ---- The tree: a tile's children are the tiles of the level below that lie in its square ----
    std::vector<std::set<std::pair<int, int>>> exists(static_cast<std::size_t>(top) + 1);
    for (int level = 0; level <= top; level++) {
        for (const auto &entry : records[static_cast<std::size_t>(level)]) {
            exists[static_cast<std::size_t>(level)].insert(entry.first);
        }
        if (level > 0) {
            for (const auto &child : exists[static_cast<std::size_t>(level) - 1]) {
                exists[static_cast<std::size_t>(level)].insert({child.first / 2, child.second / 2});
            }
        }
    }
    std::function<TilesetNode(int, int, int)> build = [&](int level, int i, int j) {
        TilesetNode node;
        bool has_box = false;
        const auto found = records[static_cast<std::size_t>(level)].find({i, j});
        if (found != records[static_cast<std::size_t>(level)].end()) {
            node.uri = found->second.uri;
            std::copy(found->second.minimum, found->second.minimum + 3, node.minimum);
            std::copy(found->second.maximum, found->second.maximum + 3, node.maximum);
            has_box = true;
        }
        node.geometric_error = level == 0 ? 0.0 : spacing * std::pow(2.0, level);
        if (level > 0) {
            for (int a = 0; a < 2; a++) {
                for (int b = 0; b < 2; b++) {
                    if (exists[static_cast<std::size_t>(level) - 1].count({2 * i + a, 2 * j + b}) == 0) {
                        continue;
                    }
                    TilesetNode child = build(level - 1, 2 * i + a, 2 * j + b);
                    for (int k = 0; k < 3; k++) {
                        node.minimum[k] = has_box ? std::min(node.minimum[k], child.minimum[k]) : child.minimum[k];
                        node.maximum[k] = has_box ? std::max(node.maximum[k], child.maximum[k]) : child.maximum[k];
                    }
                    has_box = true;
                    node.children.push_back(std::move(child));
                }
            }
        }
        return node;
    };
    if (exists[static_cast<std::size_t>(top)].size() != 1) {
        LOG_ERROR("the top level has " + std::to_string(exists[static_cast<std::size_t>(top)].size()) + " tiles, not one");
        return 1;
    }
    const std::pair<int, int> root_index = *exists[static_cast<std::size_t>(top)].begin();
    const TilesetNode root = build(top, root_index.first, root_index.second);
    const std::filesystem::path tileset_path = options.output / "tileset.json";
    if (!write_tileset_tree_json(tileset_path, frame, root, metadata, "MB-System mbglb2tiles " + std::string(MB_VERSION), &error)) {
        LOG_ERROR(error);
        return 1;
    }
    LOG_INFO("[tileset]", "wrote " + tileset_path.string() + " with " + std::to_string(tiles_written) + " tiles in " + std::to_string(top + 1) + " levels, " +
                              std::to_string(bytes_written / (1024 * 1024)) + " MiB");
    LOG_INFO("[complete]", "total " + run_timer.text() + ", " + mbmesh::peak_memory_text());
    return 0;
}
