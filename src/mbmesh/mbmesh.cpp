#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <cmath>
#include <getopt.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "settings.h"

#include "io/coordinate_frame.h"
#include "io/xyz_writer.h"
#include "io/glb_writer.h"
#include "io/x3dom_writer.h"

#include "algorithms/point_decimation.h"
#include "algorithms/normal_estimation.h"
#include "algorithms/reconstruction.h"
#include "algorithms/screened_poisson.h"
#include "algorithms/marching_cubes.h"
#include "algorithms/support_trimming.h"

// ========================================================================================================

Options parse_options(int argc, char **argv);

void print_usage();

namespace {

void log_message(const std::string &stage, const std::string &message, bool verbose, bool always_print = false) {
    if (!verbose && !always_print) {
        return;
    }
    std::cerr << "mbmesh: [" << stage << "] " << message << '\n';
}

void log_warning(const std::string &message) {
    std::cerr << "mbmesh: [warning] " << message << '\n';
}

void log_fatal(const std::string &message) {
    std::cerr << "mbmesh: [fatal] " << message << '\n';
}

} // namespace

namespace {

// Output-frame conversion: local ENU -> ECEF, minus the GeoOrigin's ECEF
// position when one is in use. Directions only rotate; they do not translate.

Vec3 output_position(const Vec3 &local, const CoordinateFrame &frame, const Vec3 &offset) {
    return local_to_ecef(local, frame) - offset;
}

Mesh output_mesh(const Mesh &mesh, const CoordinateFrame &frame, const Vec3 &offset) {
    Mesh out;
    out.indices = mesh.indices;
    out.vertices.reserve(mesh.vertices.size());
    for (const Vec3 &vertex : mesh.vertices) {
        out.vertices.push_back(output_position(vertex, frame, offset));
    }
    out.normals.reserve(mesh.normals.size());
    for (const Vec3 &normal : mesh.normals) {
        out.normals.push_back(local_vector_to_ecef(normal, frame));
    }
    return out;
}

PointCloud output_points(const PointCloud &points, const CoordinateFrame &frame, const Vec3 &offset) {
    PointCloud out;
    out.reserve(points.size());
    for (const Vec3 &point : points) {
        out.push_back(output_position(point, frame, offset));
    }
    return out;
}

OrientedPointCloud output_oriented_points(const OrientedPointCloud &points, const CoordinateFrame &frame, const Vec3 &offset) {
    OrientedPointCloud out;
    out.reserve(points.size());
    for (const OrientedPoint &point : points) {
        OrientedPoint converted(output_position(point.point, frame, offset), local_vector_to_ecef(point.normal, frame));
        converted.lambdas = point.lambdas;
        out.push_back(converted);
    }
    return out;
}

CollectedPointCloud output_collected_points(const CollectedPointCloud &points, const CoordinateFrame &frame, const Vec3 &offset) {
    CollectedPointCloud out;
    out.reserve(points.size());
    for (const CollectedPoint &point : points) {
        out.push_back({output_position(point.point, frame, offset), output_position(point.origin, frame, offset)});
    }
    return out;
}

void append_mesh(Mesh *destination, const Mesh &source) {
    const unsigned int offset = static_cast<unsigned int>(destination->vertices.size());
    const bool had_normals = destination->normals.size() == destination->vertices.size();
    const bool has_normals = source.normals.size() == source.vertices.size();
    destination->vertices.insert(destination->vertices.end(), source.vertices.begin(), source.vertices.end());
    if (had_normals && has_normals) {
        destination->normals.insert(destination->normals.end(), source.normals.begin(), source.normals.end());
    } else {
        destination->normals.clear();
    }
    destination->indices.reserve(destination->indices.size() + source.indices.size());
    for (const unsigned int index : source.indices) {
        destination->indices.push_back(index + offset);
    }
}

} // namespace

// Where output vertices go. GLB outputs are WGS84 ECEF; with a GeoOrigin they
// are offsets from the GeoOrigin's ECEF position (exactly as in mbgrd2gltf).
struct OutputPlacement {
    bool use_geoorigin = false;
    GeodeticPosition geoorigin;
    X3DomGeoOrigin view;
};

bool write_outputs(
    const Mesh &mesh,
    const Mesh &raw_mesh,
    const PointCloud &points,
    const OrientedPointCloud &oriented_points,
    const CollectedPointCloud &collected_points,
    const CoordinateFrame &frame,
    const OutputPlacement &placement,
    const Options &options,
    std::string *error);

bool launch_html_viewer_server(
    const std::filesystem::path &directory,
    const std::string &html_filename);

// ========================================================================================================

int main(int argc, char **argv) {

    Options options = parse_options(argc, argv);

    if (options.help_requested) {
        print_usage();
        return 0;
    }

    log_message("startup",
                "input=" + options.input_datalist.string() + " output=" + options.output_directory.string(),
                options.verbose,
                true);

    if (options.input_datalist.empty()) {
        log_fatal("input datalist path is empty; use -I <path>");
        return 1;
    }

    log_message("input", "reading datalist " + options.input_datalist.string(), options.verbose);

    std::string error;
    PreprocessedDatalist preprocessed;
    if (!preprocess_datalist(options, &preprocessed, &error)) {
        log_fatal(error);
        return 1;
    }

    log_message("input", "completed: accepted " + std::to_string(preprocessed.read_result.points.size()) +
                              " of " + std::to_string(preprocessed.read_result.stats.soundings_read) +
                              " soundings from " + std::to_string(preprocessed.read_result.stats.files_read) +
                              " files",
                options.verbose);

    if (options.metadata_requested) {
        print_datalist_metadata(preprocessed, options);
        return 0;
    }

    // ====================================================================================================
    // Output Placement (--geoorigin)
    // ====================================================================================================

    OutputPlacement placement;
    {
        const GeodeticBounds &bounds = preprocessed.metadata.geodetic_bounds;
        placement.use_geoorigin = resolve_geoorigin(options, preprocessed.metadata, &placement.geoorigin);

        if (placement.use_geoorigin) {
            std::ostringstream message;
            message << std::fixed << std::setprecision(6)
                    << (options.geoorigin_mode == GeoOriginMode::Auto ? "using automatic GeoOrigin (data center): "
                                                                       : "using user-specified GeoOrigin: ")
                    << placement.geoorigin.longitude << "," << placement.geoorigin.latitude << ","
                    << std::setprecision(2) << placement.geoorigin.elevation;
            log_message("output", message.str(), options.verbose);

            // Viewpoint values follow mbgrd2gltf: center of the bounds, framed to fill a 45 degree view.
            X3DomGeoOrigin &view = placement.view;
            view.origin_lat = placement.geoorigin.latitude;
            view.origin_lon = placement.geoorigin.longitude;
            view.origin_elevation = placement.geoorigin.elevation;
            view.center_lat = (bounds.min_lat + bounds.max_lat) / 2.0;
            view.center_lon = (bounds.min_lon + bounds.max_lon) / 2.0;
            view.mean_elevation = preprocessed.metadata.mean_elevation;
            constexpr double pi = 3.14159265358979323846;
            const double lat_extent_m = (bounds.max_lat - bounds.min_lat) * 111000.0;
            const double lon_extent_m = (bounds.max_lon - bounds.min_lon) * 111000.0 * std::cos(view.center_lat * pi / 180.0);
            constexpr double field_of_view = 0.785398;
            view.view_altitude = (std::max(lat_extent_m, lon_extent_m) / 2.0) / std::tan(field_of_view / 2.0);
        } else {
            log_warning("no GeoOrigin: GLB vertices are absolute ECEF stored as 32-bit floats (about 0.5 m resolution); "
                        "use --geoorigin to preserve sub-meter detail");
        }
    }

    CollectedPointCloud collected_points = std::move(preprocessed.read_result.points);

    // ====================================================================================================
    // Point Decimation
    // ====================================================================================================

    const std::size_t input_sample_count = collected_points.size();

    CollectedPointCloud decimated_points;

    if (options.decimation.decimate) {

        log_message("decimation", "starting: cell_size=" + std::to_string(options.decimation.cell_size), options.verbose);

        decimated_points = point_decimation(std::move(collected_points), options.decimation);

        if (decimated_points.empty()) {
            log_fatal("point decimation produced no samples");
            return 1;
        }
        log_message("decimation",
                    "completed: retained " + std::to_string(decimated_points.size()) + " of " +
                        std::to_string(input_sample_count) + " samples",
                    options.verbose);

    } else {
        log_message("decimation",
                    "disabled; using " + std::to_string(input_sample_count) + " input samples",
                    options.verbose);
        decimated_points = std::move(collected_points);
    }

    // ====================================================================================================
    // Reconstruction: normal estimation, screened Poisson, marching cubes, support trimming
    // ====================================================================================================

    const ReconstructionOptions reconstruction = reconstruction_options(options);
    const ReconstructionLogger reconstruction_log = [&](const std::string &stage, const std::string &message) {
        log_message(stage, message, options.verbose);
    };

    Mesh clean_mesh;
    Mesh raw_mesh;
    OrientedPointCloud oriented_points;

    if (!options.use_chunking) {
        ReconstructionResult result = reconstruct_chunk(decimated_points, reconstruction, reconstruction_log);
        if (result.status != ReconstructionStatus::Ok) {
            log_fatal(reconstruction_status_message(result.status));
            return 1;
        }
        clean_mesh = std::move(result.mesh);
        raw_mesh = std::move(result.raw_mesh);
        oriented_points = std::move(result.oriented_points);
    }
    else {
        const bool keep_raw_mesh = options.write_raw_mesh_glb;
        const bool keep_oriented_points = options.write_normal_glb || options.write_oriented_ply;

        const std::filesystem::path tiles_directory = options.output_directory / "tiles";
        std::error_code filesystem_error;
        std::filesystem::create_directories(tiles_directory, filesystem_error);
        if (filesystem_error) {
            log_fatal("error creating tile directory '" + tiles_directory.string() + "': " + filesystem_error.message());
            return 1;
        }

        const Vec3 tile_offset = placement.use_geoorigin ? geodetic_to_ecef(placement.geoorigin) : Vec3(0.0, 0.0, 0.0);
        const CoordinateFrame &tile_frame = preprocessed.read_result.frame;

        const TileCallback on_tile = [&](TileResult &tile) {
            const std::string tile_name = "tile_" + std::to_string(tile.i) + "_" + std::to_string(tile.j) + ".glb";
            std::string tile_error;
            if (!write_mesh_glb_file(tiles_directory / tile_name, output_mesh(tile.result.mesh, tile_frame, tile_offset), &tile_error)) {
                log_warning(tile_error);
            }

            append_mesh(&clean_mesh, tile.result.mesh);
            if (keep_raw_mesh) {
                append_mesh(&raw_mesh, tile.result.raw_mesh);
            }
            if (keep_oriented_points) {
                oriented_points.insert(oriented_points.end(), tile.result.oriented_points.begin(), tile.result.oriented_points.end());
            }
            return true;
        };

        ChunkingStats chunking_stats;
        if (!reconstruct_chunked(decimated_points, reconstruction, options.chunking, reconstruction_log, on_tile, &chunking_stats, &error)) {
            log_fatal(error);
            return 1;
        }

        log_message("chunking",
                    "completed: " + std::to_string(chunking_stats.tiles_processed) + " tiles written, " +
                        std::to_string(chunking_stats.tiles_failed) + " failed, " +
                        std::to_string(chunking_stats.tiles_skipped_sparse) + " skipped as sparse, " +
                        std::to_string(chunking_stats.tiles_empty_after_crop) + " empty",
                    options.verbose, true);

        if (clean_mesh.vertices.empty() || clean_mesh.indices.empty()) {
            log_fatal("chunked reconstruction produced no mesh; adjust the level of detail or chunk size");
            return 1;
        }
    }

    // ====================================================================================================
    // Write Outputs
    // ====================================================================================================

    if (options.verbose) {
        std::ostringstream verbose_message;
        verbose_message << "starting: directory=" << options.output_directory
                        << ", glb=enabled"
                        << ", html=" << (options.write_html ? "enabled" : "disabled")
                        << ", local_xyz=" << (options.write_local_xyz ? "enabled" : "disabled")
                        << ", ecef_xyz=" << (options.write_ecef_xyz ? "enabled" : "disabled")
                        << ", oriented_ply=" << (options.write_oriented_ply ? "enabled" : "disabled")
                        << ", pointcloud_glb=" << (options.write_pointcloud_glb ? "enabled" : "disabled")
                        << ", normal_glb=" << (options.write_normal_glb ? "enabled" : "disabled")
                        << ", origin_glb=" << (options.write_origin_glb ? "enabled" : "disabled")
                        << ", raw_mesh_glb=" << (options.write_raw_mesh_glb ? "enabled" : "disabled");
        log_message("output", verbose_message.str(), options.verbose);
    }

    PointCloud output_points;
    output_points.reserve(decimated_points.size());
    for (const CollectedPoint &collected_point : decimated_points) {
        output_points.push_back(collected_point.point);
    }

    if (!write_outputs(clean_mesh, raw_mesh, output_points, oriented_points, decimated_points, preprocessed.read_result.frame, placement, options, &error)) {
        log_fatal(error);
        return 1;
    }

    log_message("output", "completed", options.verbose);
    log_message("complete", "wrote " + (options.output_directory / "mesh.glb").string(), options.verbose, true);

    // ====================================================================================================
    // Launch Local Server for X3DOM Viewer
    // ====================================================================================================

    if (options.write_html && !launch_html_viewer_server(options.output_directory, "mesh.html")) {
        log_warning("failed to auto-launch Python web server/viewer");
    }

    return 0;
}

// ========================================================================================================

Options parse_options(int argc, char **argv) {

    Options options;

    struct SingleDashLongOption {
        const char *single_dash;
        const char *double_dash;
    };

    static const SingleDashLongOption single_dash_long_options[] = {
        {"-html", "--html"},
        {"-local-xyz", "--local-xyz"},
        {"-ecef-xyz", "--ecef-xyz"},
        {"-xyz", "--xyz"},
        {"-oriented-ply", "--oriented-ply"},
        {"-pointcloud-glb", "--pointcloud-glb"},
        {"-normal-glb", "--normal-glb"},
        {"-origin-glb", "--origin-glb"},
        {"-raw-mesh-glb", "--raw-mesh-glb"},
        {"-decimate", "--decimate"},
        {"-diagnostics", "--diagnostics"},
        {"-all-outputs", "--all-outputs"},
    };

    for (int i = 1; i < argc; i++) {
        for (const SingleDashLongOption &option_alias : single_dash_long_options) {
            if (std::strcmp(argv[i], option_alias.single_dash) == 0) {
                argv[i] = const_cast<char *>(option_alias.double_dash);
                break;
            }
        }
    }

    static const struct option long_options[] = {
        {"input", required_argument, nullptr, 'I'},
        {"output", required_argument, nullptr, 'O'},
        {"bounds", required_argument, nullptr, 'R'},
        {"lod", required_argument, nullptr, 'L'},
        {"level-of-detail", required_argument, nullptr, 'L'},
        {"metadata", no_argument, nullptr, 1000},
        {"info", no_argument, nullptr, 1000},
        {"html", no_argument, nullptr, 1001},
        {"local-xyz", no_argument, nullptr, 1002},
        {"ecef-xyz", no_argument, nullptr, 1003},
        {"xyz", no_argument, nullptr, 1004},
        {"oriented-ply", no_argument, nullptr, 1005},
        {"pointcloud-glb", no_argument, nullptr, 1006},
        {"normal-glb", no_argument, nullptr, 1007},
        {"origin-glb", no_argument, nullptr, 1008},
        {"raw-mesh-glb", no_argument, nullptr, 1009},
        {"decimate", required_argument, nullptr, 1010},
        {"geoorigin", optional_argument, nullptr, 'G'},
        {"chunk-size", required_argument, nullptr, 1013},
        {"chunk", required_argument, nullptr, 1014},
        {"max-chunks", required_argument, nullptr, 1015},
        {"threads", required_argument, nullptr, 1016},
        {"diagnostics", no_argument, nullptr, 1011},
        {"all-outputs", no_argument, nullptr, 1012},
        {"verbose", no_argument, nullptr, 'V'},
        {"help", no_argument, nullptr, 'H'},
        {nullptr, 0, nullptr, 0},
    };

    int c = 0;
    int option_index = 0;
    while ((c = getopt_long(argc, argv, "I:O:R:L:G::VHh", long_options, &option_index)) != -1) {

        switch (c) {

        case 'I':
            options.input_datalist = optarg;
            break;

        case 'O':
            options.output_directory = optarg;
            break;

        case 'R': {
            double west = 0.0, east = 0.0, south = 0.0, north = 0.0;
            if (std::sscanf(optarg, "%lf/%lf/%lf/%lf", &west, &east, &south, &north) == 4) {
                options.bounds.degrees_W = west;
                options.bounds.degrees_E = east;
                options.bounds.degrees_S = south;
                options.bounds.degrees_N = north;
                options.use_bounds = true;
            } else {
                log_warning("invalid bounds argument: " + std::string(optarg));
            }
            break;
        }

        case 'L':
            options.level_of_detail = std::strtod(optarg, nullptr);
            options.level_of_detail_requested = true;
            break;

        case 'G':
            // Same syntax and meaning as mbgrd2gltf: no value selects the
            // automatic GeoOrigin; otherwise all of lon,lat,elev are required.
            // As in mbgrd2gltf, the value must be attached (--geoorigin=V, -GV).
            if (optarg == nullptr) {
                options.geoorigin_mode = GeoOriginMode::Auto;
            } else {
                const std::string geoorigin_string(optarg);
                const std::size_t comma1 = geoorigin_string.find(',');
                const std::size_t comma2 = geoorigin_string.rfind(',');
                if (comma1 == std::string::npos || comma2 == std::string::npos || comma1 == comma2) {
                    log_fatal("GeoOrigin format must be lon,lat,elev");
                    std::exit(1);
                }
                options.geoorigin.longitude = std::atof(geoorigin_string.substr(0, comma1).c_str());
                options.geoorigin.latitude = std::atof(geoorigin_string.substr(comma1 + 1, comma2 - comma1 - 1).c_str());
                options.geoorigin.elevation = std::atof(geoorigin_string.substr(comma2 + 1).c_str());
                options.geoorigin_mode = GeoOriginMode::Specified;
            }
            break;

        case 'V':
            options.verbose = true;
            break;

        case 1000:
            options.metadata_requested = true;
            break;

        case 1001:
            options.write_html = true;
            break;

        case 1002:
            options.write_local_xyz = true;
            break;

        case 1003:
            options.write_ecef_xyz = true;
            break;

        case 1004:
            options.write_local_xyz = true;
            options.write_ecef_xyz = true;
            break;

        case 1005:
            options.write_oriented_ply = true;
            break;

        case 1006:
            options.write_pointcloud_glb = true;
            break;

        case 1007:
            options.write_normal_glb = true;
            break;

        case 1008:
            options.write_origin_glb = true;
            break;

        case 1009:
            options.write_raw_mesh_glb = true;
            break;

        case 1010:
            options.decimation.decimate = true;
            options.decimation.cell_size = std::strtod(optarg, nullptr);
            options.decimation_requested = true;
            break;

        case 1013:
            options.chunk_size = std::strtod(optarg, nullptr);
            options.chunk_size_requested = true;
            break;

        case 1014: {
            int tile_i = 0, tile_j = 0;
            if (std::sscanf(optarg, "%d,%d", &tile_i, &tile_j) == 2) {
                options.chunk_tile_i = tile_i;
                options.chunk_tile_j = tile_j;
                options.chunk_tile_requested = true;
            } else {
                log_fatal("invalid --chunk argument (expected I,J): " + std::string(optarg));
                std::exit(1);
            }
            break;
        }

        case 1015:
            options.max_chunks = static_cast<std::size_t>(std::strtoull(optarg, nullptr, 10));
            break;

        case 1016:
            options.threads = static_cast<std::size_t>(std::strtoull(optarg, nullptr, 10));
            break;

        case 1011:
            options.write_pointcloud_glb = true;
            options.write_normal_glb = true;
            options.write_origin_glb = true;
            break;

        case 1012:
            options.write_html = true;
            options.write_local_xyz = true;
            options.write_ecef_xyz = true;
            options.write_oriented_ply = true;
            options.write_pointcloud_glb = true;
            options.write_normal_glb = true;
            options.write_origin_glb = true;
            options.write_raw_mesh_glb = true;
            break;

        case 'H':
        case 'h':
            options.help_requested = true;
            break;

        default:
            break;
        }
    }

    if (options.input_datalist.empty() && optind < argc) {
        options.input_datalist = argv[optind];
    }

    return options;
}

void print_usage() {
    std::cout << "mbmesh generates 3D meshes from swath sonar bathymetry data.\n\n";
    std::cout << "usage: mbmesh -I datalist [-R west/east/south/north] [-O outputdir] [-L meters] [output options] [-V]\n\n";
    std::cout << "Required:\n";
    std::cout << "  -I, --input <datalist>       Input MB-System datalist file\n\n";
    std::cout << "Optional:\n";
    std::cout << "  -O, --output <outputdir>     Output directory [mbmesh_output]\n";
    std::cout << "  -R, --bounds <w/e/s/n>       Geographic bounds in degrees\n";
    std::cout << "  -L, --lod <meters>           Requested smallest feature size; auto if omitted\n";
    std::cout << "      --decimate <meters>      Enable voxel-grid point decimation\n";
    std::cout << "  -G, --geoorigin [lon,lat,elev]\n";
    std::cout << "                               Write vertices as ECEF offsets from a GeoOrigin, as in mbgrd2gltf\n";
    std::cout << "                                 With values: use the specified lon,lat,elev\n";
    std::cout << "                                 Without values: use the data center and mean elevation\n";
    std::cout << "                                 Not specified: absolute ECEF coordinates (default)\n";
    std::cout << "                               GLB positions are 32-bit floats, so absolute ECEF (magnitudes of about\n";
    std::cout << "                               6.4 million meters) resolves only about 0.5 m; use a GeoOrigin to\n";
    std::cout << "                               preserve sub-meter detail\n";
    std::cout << "                               The value must be attached: --geoorigin=lon,lat,elev or -Glon,lat,elev\n";
    std::cout << "      --chunk-size <meters>    Reconstruct in square tiles of about this size, writing tiles/tile_I_J.glb\n";
    std::cout << "                               plus the merged mesh.glb; chunking turns on automatically for\n";
    std::cout << "                               surveys too large for one Poisson grid\n";
    std::cout << "      --chunk <i,j>            Reconstruct only tile (i,j); tile (i,j) covers x in [i*size,(i+1)*size)\n";
    std::cout << "                               and y in [j*size,(j+1)*size) in the local east-north frame\n";
    std::cout << "      --max-chunks <n>         Stop after n tiles (for testing)\n";
    std::cout << "      --threads <n>            Tiles reconstructed at once when chunking [min(cores,4)]; each holds its own grid\n";
    std::cout << "      --metadata, --info       Print dataset metadata and exit\n";
    std::cout << "  -V, --verbose                Enable progress and diagnostic logging\n";
    std::cout << "  -H, -h, --help               Print this help message\n\n";
    std::cout << "Output options:\n";
    std::cout << "      --html                   Also write mesh.html and launch local X3DOM viewer\n";
    std::cout << "      --xyz                    Also write local and ECEF XYZ point clouds\n";
    std::cout << "      --local-xyz              Also write pointcloud-local.xyz\n";
    std::cout << "      --ecef-xyz               Also write pointcloud-ecef.xyz\n";
    std::cout << "      --oriented-ply           Also write oriented-pointcloud-ecef.ply\n";
    std::cout << "      --pointcloud-glb         Also write pointcloud.glb\n";
    std::cout << "      --normal-glb             Also write normals.glb\n";
    std::cout << "      --origin-glb             Also write origins.glb\n";
    std::cout << "      --raw-mesh-glb           Also write raw_mesh.glb before support trimming\n";
    std::cout << "      --diagnostics            Also write pointcloud, normal, and origin GLBs\n";
    std::cout << "      --all-outputs            Write all optional outputs\n\n";
    std::cout << "The default output is mesh.glb.\n";
    std::cout << "Single-dash long output options such as -html are accepted for legacy compatibility.\n";
}

namespace {

constexpr std::size_t maximum_diagnostic_glb_points = 2'000'000;

std::string shell_quote(const std::string &value) {
    std::string quoted = "'";
    for (const char character : value) {
        if (character == '\'') {
            quoted += "'\\''";
        } else {
            quoted += character;
        }
    }
    quoted += "'";
    return quoted;
}

} // namespace

// ========================================================================================================

bool write_outputs(
    const Mesh &mesh,
    const Mesh &raw_mesh,
    const PointCloud &points,
    const OrientedPointCloud &oriented_points,
    const CollectedPointCloud &collected_points,
    const CoordinateFrame &frame,
    const OutputPlacement &placement,
    const Options &options,
    std::string *error)
{
    const Vec3 offset = placement.use_geoorigin ? geodetic_to_ecef(placement.geoorigin) : Vec3(0.0, 0.0, 0.0);

    if (options.output_directory.empty()) {
        if (error != nullptr) {
            *error = "Output directory path is empty";
        }
        return false;
    }

    std::error_code filesystem_error;
    std::filesystem::create_directories(options.output_directory, filesystem_error);

    if (filesystem_error) {
        if (error != nullptr) {
            *error = "Error creating output directory '" + options.output_directory.string() + "': " + filesystem_error.message();
        }
        return false;
    }

    if (options.write_ecef_xyz) {
        const auto ecef_xyz_path = options.output_directory / "pointcloud-ecef.xyz";
        if (!write_ecef_xyz_pointcloud(points, frame, ecef_xyz_path.string(), error)) {
            return false;
        }
        log_message("output", "wrote " + ecef_xyz_path.string(), options.verbose);
    }

    if (options.write_local_xyz) {
        const auto local_xyz_path = options.output_directory / "pointcloud-local.xyz";
        if (!write_local_xyz_pointcloud(points, local_xyz_path.string(), error)) {
            return false;
        }
        log_message("output", "wrote " + local_xyz_path.string(), options.verbose);
    }

    if (options.write_oriented_ply) {
        const auto oriented_ply_path = options.output_directory / "oriented-pointcloud-ecef.ply";
        if (!write_ply_oriented_pointcloud(oriented_points, frame, oriented_ply_path.string(), error)) {
            return false;
        }
        log_message("output", "wrote " + oriented_ply_path.string(), options.verbose);
    }

    if (options.write_pointcloud_glb) {
        const auto pointcloud_glb_path = options.output_directory / "pointcloud.glb";
        if (!write_pointcloud_glb_file(pointcloud_glb_path, output_points(points, frame, offset), error)) {
            return false;
        }
        log_message("output", "wrote " + pointcloud_glb_path.string(), options.verbose);
    }

    if (options.write_normal_glb) {
        const auto normals_glb_path = options.output_directory / "normals.glb";
        if (!write_normal_lines_glb_file(normals_glb_path, output_oriented_points(oriented_points, frame, offset), 0.25, error)) {
            return false;
        }
        log_message("output", "wrote " + normals_glb_path.string(), options.verbose);
    }

    if (options.write_origin_glb) {
        const auto origins_glb_path = options.output_directory / "origins.glb";
        if (!write_origin_ray_lines_glb_file(origins_glb_path, output_collected_points(collected_points, frame, offset), 0.25, error)) {
            return false;
        }
        log_message("output", "wrote " + origins_glb_path.string(), options.verbose);
    }

    if (options.write_raw_mesh_glb) {
        const auto raw_mesh_glb_path = options.output_directory / "raw_mesh.glb";
        if (!write_mesh_glb_file(raw_mesh_glb_path, output_mesh(raw_mesh, frame, offset), error)) {
            return false;
        }
        log_message("output", "wrote " + raw_mesh_glb_path.string(), options.verbose);
    }

    const auto clean_mesh_glb_path = options.output_directory / "mesh.glb";
    if (!write_mesh_glb_file(clean_mesh_glb_path, output_mesh(mesh, frame, offset), error)) {
        return false;
    }
    log_message("output", "wrote " + clean_mesh_glb_path.string(), options.verbose);

    if (options.write_html) {
        const auto html_path = options.output_directory / "mesh.html";
        X3DomWriterOptions html_options;
        html_options.use_geoorigin = placement.use_geoorigin;
        html_options.geo = placement.view;
        if (!write_glb_x3dom_file(html_path, "mesh.glb", html_options, error)) {
            return false;
        }
        log_message("output", "wrote " + html_path.string(), options.verbose);
    }

    return true;
}

// ========================================================================================================

bool launch_html_viewer_server(
    const std::filesystem::path &directory,
    const std::string &html_filename)
{
    if (directory.empty() || html_filename.empty()) {
        return false;
    }

    const int port = 8000;
    const std::string directory_string = directory.string();

    std::string server_command =
        "python3 -m http.server " + std::to_string(port) +
        " --bind 127.0.0.1 --directory " + shell_quote(directory_string) +
        " >/tmp/mbmesh_http.log 2>&1 &";

    const int server_status = std::system(server_command.c_str());
    if (server_status != 0) {
        return false;
    }

    const std::string url =
        "http://127.0.0.1:" + std::to_string(port) + "/" + html_filename;
    const std::string open_command =
        "python3 -c \"import webbrowser; webbrowser.open('" + url + "')\"";

    const int open_status = std::system(open_command.c_str());
    if (open_status != 0) {
        return false;
    }

    std::cerr << "mbmesh: [output] started Python web server at " << url << '\n';
    std::cerr << "mbmesh: [output] server logs: /tmp/mbmesh_http.log\n";
    return true;
}
