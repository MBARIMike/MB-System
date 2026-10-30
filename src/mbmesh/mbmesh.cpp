#include <algorithm>
#include <cstdio>
#include <ctime>
#include <unistd.h>
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

#include "util/logger.h"
#include "util/stopwatch.h"

#include "io/coordinate_frame.h"
#include "io/xyz_writer.h"
#include "io/glb_writer.h"
#include "io/tileset_writer.h"
#include "io/track.h"
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
    const std::vector<TrackPoint> &track,
    const Options &options,
    std::string *error);

void print_html_viewer_command(
    const std::filesystem::path &directory,
    const std::string &html_filename);

// ========================================================================================================

namespace {

// The page of a directory input: every GLB beneath it. Returns false after logging an error.
bool write_survey_page(const std::filesystem::path &survey_directory, const std::string &x3dom_url, const std::vector<std::string> &geoviewpoints,
                       double lift_meters) {
    const std::filesystem::path html_path = survey_page_path();
    X3DomWriterOptions html_options;
    html_options.title = viewer_title(survey_directory);
    html_options.nested_groups = true;
    html_options.lift_meters = lift_meters;
    if (!x3dom_url.empty()) {
        html_options.x3dom_url = x3dom_url;
    }
    std::string error;
    std::vector<std::string> mismatches;
    html_options.models = find_survey_models(survey_directory, html_path.parent_path(), &html_options.use_geoorigin, &html_options.geo, &mismatches);
    for (const std::string &mismatch : mismatches) {
        LOG_ERROR("[html] GeoOrigin mismatch: " + mismatch);
    }
    if (html_options.models.empty()) {
        LOG_ERROR("no .glb files beneath " + survey_directory.string());
        return false;
    }
    if (!html_options.models.front().uri.empty() && html_options.models.front().uri.rfind("../", 0) == 0) {
        LOG_WARN("the directory is not beneath the current directory, so a web server run here cannot serve its GLBs; run mbmesh in the survey directory or above it");
    }
    for (const std::string &text : geoviewpoints) {
        X3DomUserViewpoint viewpoint;
        std::string viewpoint_error;
        if (!parse_user_viewpoint(text, &viewpoint, &viewpoint_error)) {
            LOG_ERROR(viewpoint_error);
            return false;
        }
        html_options.user_viewpoints.push_back(viewpoint);
    }
    html_options.log_messages = mbmesh::Logger::captured_messages();
    html_options.log_messages_omitted = mbmesh::Logger::captured_omitted();
    if (!write_glb_x3dom_file(html_path, html_options, &error)) {
        LOG_ERROR(error);
        return false;
    }
    LOG_INFO("[html]", "wrote " + html_path.string() + ", offering " + std::to_string(html_options.models.size()) + " GLB files");
    print_html_viewer_command(html_path.parent_path(), html_path.filename().string());
    return true;
}

} // namespace

int main(int argc, char **argv) {
    const std::time_t started_at = std::time(nullptr);

    using mbmesh::LogLevel;
    using mbmesh::Logger;
    using mbmesh::Stopwatch;

    const Stopwatch run_timer;

    // INFO until the options are parsed, so option errors are reported; -V then
    // raises it to DEBUG for progress messages, as in mbgrd2gltf.
    Logger::set_level(LogLevel::INFO);

    Options options = parse_options(argc, argv);
    if (options.verbose) {
        Logger::set_level(LogLevel::DEBUG);
    }

    if (options.help_requested) {
        print_usage();
        return 0;
    }

    // Keep the log for the processing log in the HTML page.
    if (options.write_html) {
        Logger::start_capture();
    }

    {
        std::string command_line;
        for (int i = 0; i < argc; i++) {
            const std::string argument = argv[i];
            if (i > 0) {
                command_line += ' ';
            }
            command_line += argument.find(' ') == std::string::npos ? argument : "'" + argument + "'";
        }
        LOG_INFO("[command]", command_line);
    }

    LOG_INFO("[startup]", "input=" + options.input_datalist.string() + " output=" + options.output_directory.string());

    if (options.input_datalist.empty()) {
        LOG_ERROR("input datalist path is empty; use -I <path>");
        return 1;
    }

    {
        std::error_code directory_error;
        if (std::filesystem::is_directory(options.input_datalist, directory_error)) {
            return write_survey_page(options.input_datalist, options.x3dom_url, options.geoviewpoints, options.lift_meters) ? 0 : 1;
        }
    }

    LOG_DEBUG("[input]", "reading datalist " + options.input_datalist.string());

    std::string error;
    PreprocessedDatalist preprocessed;
    const Stopwatch input_timer;
    if (!preprocess_datalist(options, &preprocessed, &error)) {
        LOG_ERROR(error);
        return 1;
    }

    LOG_INFO("[input]", "completed: accepted " + std::to_string(preprocessed.read_result.points.size()) +
                            " of " + std::to_string(preprocessed.read_result.stats.soundings_read) +
                            " soundings from " + std::to_string(preprocessed.read_result.stats.files_read) +
                            " files in " + input_timer.text());

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

        // Viewpoint values follow mbgrd2gltf: center of the bounds, framed to fill a 45 degree view.
        X3DomGeoOrigin &view = placement.view;
        view.center_lat = (bounds.min_lat + bounds.max_lat) / 2.0;
        view.center_lon = (bounds.min_lon + bounds.max_lon) / 2.0;
        view.mean_elevation = preprocessed.metadata.mean_elevation;
        constexpr double pi = 3.14159265358979323846;
        const double lat_extent_m = (bounds.max_lat - bounds.min_lat) * 111000.0;
        const double lon_extent_m = (bounds.max_lon - bounds.min_lon) * 111000.0 * std::cos(view.center_lat * pi / 180.0);
        constexpr double field_of_view = 0.785398;
        view.view_altitude = (std::max(lat_extent_m, lon_extent_m) / 2.0) / std::tan(field_of_view / 2.0);

        // The viewpoint in the coordinates of the GLBs: ECEF, minus the GeoOrigin's ECEF position if there is one.
        {
            GeodeticPosition middle;
            middle.longitude = view.center_lon;
            middle.latitude = view.center_lat;
            middle.elevation = view.mean_elevation;
            const Vec3 middle_ecef = geodetic_to_ecef(middle);
            const Vec3 origin_ecef = placement.use_geoorigin ? geodetic_to_ecef(placement.geoorigin) : Vec3(0.0, 0.0, 0.0);
            const double middle_array[3] = {middle_ecef.x, middle_ecef.y, middle_ecef.z};
            const double origin_array[3] = {origin_ecef.x, origin_ecef.y, origin_ecef.z};
            set_local_viewpoint(&view, middle_array, origin_array);
        }

        if (placement.use_geoorigin) {
            std::ostringstream message;
            message << std::fixed << std::setprecision(6)
                    << (options.geoorigin_mode == GeoOriginMode::Auto ? "using automatic GeoOrigin (data center): "
                                                                       : "using user-specified GeoOrigin: ")
                    << placement.geoorigin.longitude << "," << placement.geoorigin.latitude << ","
                    << std::setprecision(2) << placement.geoorigin.elevation;
            LOG_INFO("[output]", message.str());

            view.origin_lat = placement.geoorigin.latitude;
            view.origin_lon = placement.geoorigin.longitude;
            view.origin_elevation = placement.geoorigin.elevation;
        } else {
            LOG_WARN("no GeoOrigin: GLB vertices are absolute ECEF stored as 32-bit floats (about 0.5 m resolution); "
                        "use --geoorigin to preserve sub-meter detail");
        }
    }

    // The settings the reconstruction will use, for the record.
    {
        std::ostringstream settings;
        settings << std::setprecision(6) << "level of detail " << options.level_of_detail << " m, Poisson cell " << options.poisson.cell_size
                 << " m, normal radius " << options.normals.search_radius << " m, point spacing " << preprocessed.metadata.spacing.median
                 << " m (median), " << preprocessed.metadata.point_count << " soundings; Poisson solver " << poisson_solver_name(options.poisson_solver)
                 << ", ";
        if (const int iterations = reconstruction_options(options).poisson.solver_iterations; iterations > 0) {
            settings << iterations << " iterations; ";
        } else {
            settings << "iterations chosen from the grid size; ";
        }
        if (options.use_chunking) {
            settings << "chunked reconstruction, tile " << options.chunking.tile_size << " m, halo " << options.chunking.halo << " m, "
                     << options.chunking.threads << " threads";
        } else {
            settings << "single-pass reconstruction";
        }
        LOG_INFO("[settings]", settings.str());
    }

    // What is recorded in every GLB and in the HTML page about where the models are
    // and how they were made. One tree, so that the files cannot disagree.
    RunInfo run;
    {
        const GeodeticBounds &bounds = preprocessed.metadata.geodetic_bounds;
        const DatalistReadResult &read_result = preprocessed.read_result;

        run.title = viewer_title(viewer_web_root(options.output_directory));
        run.version = mbsystem_version();
        {
            std::string command_line;
            for (int i = 0; i < argc; i++) {
                const std::string argument = argv[i];
                command_line += (i > 0 ? " " : "") + (argument.find(' ') == std::string::npos ? argument : "'" + argument + "'");
            }
            run.command_line = command_line;
        }
        std::error_code cwd_error;
        run.working_directory = std::filesystem::current_path(cwd_error).string();
        run.started_at = iso_utc_time(static_cast<double>(started_at));
        if (const char *user = std::getenv("USER")) {
            run.user = user;
        } else if (const char *user_name = std::getenv("LOGNAME")) {
            run.user = user_name;
        }
        char host[256] = "";
        if (gethostname(host, sizeof(host) - 1) == 0) {
            run.host = host;
        }

        std::error_code absolute_error;
        const std::filesystem::path datalist = std::filesystem::absolute(options.input_datalist, absolute_error);
        run.datalist = absolute_error ? options.input_datalist.string() : datalist.lexically_normal().string();
        run.formats = read_result.formats;
        run.file_count = read_result.files.size();
        run.files.assign(read_result.files.begin(), read_result.files.begin() + std::min(read_result.files.size(), maximum_listed_files));
        run.pings = read_result.stats.pings_read;
        run.soundings_read = read_result.stats.soundings_read;
        run.soundings_accepted = read_result.stats.soundings_accepted;
        run.soundings_flagged = read_result.stats.soundings_flagged;
        run.soundings_out_of_bounds = read_result.stats.soundings_out_of_bounds;
        run.bounds_applied = options.use_bounds;
        run.bounds_west = options.bounds.degrees_W;
        run.bounds_east = options.bounds.degrees_E;
        run.bounds_south = options.bounds.degrees_S;
        run.bounds_north = options.bounds.degrees_N;
        if (!read_result.track.empty()) {
            const auto times = std::minmax_element(read_result.track.begin(), read_result.track.end(),
                                                   [](const TrackPoint &a, const TrackPoint &b) { return a.time < b.time; });
            run.first_time = iso_utc_time(times.first->time);
            run.last_time = iso_utc_time(times.second->time);
        }

        run.geoorigin_mode = !placement.use_geoorigin ? "none" : (options.geoorigin_mode == GeoOriginMode::Auto ? "automatic" : "specified");
        if (placement.use_geoorigin) {
            const Vec3 center = geodetic_to_ecef(placement.geoorigin);
            run.has_center = true;
            run.center_x = center.x;
            run.center_y = center.y;
            run.center_z = center.z;
            run.center_longitude = placement.geoorigin.longitude;
            run.center_latitude = placement.geoorigin.latitude;
            run.center_height = placement.geoorigin.elevation;
        }
        run.extent_west = bounds.min_lon;
        run.extent_south = bounds.min_lat;
        run.extent_east = bounds.max_lon;
        run.extent_north = bounds.max_lat;
        run.mean_elevation = preprocessed.metadata.mean_elevation;

        run.level_of_detail = options.level_of_detail;
        run.poisson_cell_size = options.poisson.cell_size;
        run.poisson_solver = poisson_solver_name(options.poisson_solver);
        run.poisson_iterations = reconstruction_options(options).poisson.solver_iterations;   // 0: chosen from each grid
        run.normal_search_radius = options.normals.search_radius;
        run.point_spacing = preprocessed.metadata.spacing.median;
        run.chunked = options.use_chunking;
        run.tile_size = options.chunking.tile_size;
        run.halo = options.chunking.halo;
        run.threads = options.use_chunking ? static_cast<std::size_t>(options.chunking.threads) : 1;
    }
    set_glb_run_metadata(run_metadata(run));

    // The OGC 3D Tiles tileset: each tile's surface in the local east-north-up frame, written Y-up as glTF requires,
    // and tileset.json, whose root transform is that frame in ECEF. It does not depend on the GeoOrigin.
    std::vector<TilesetTile> tileset_tiles;
    MetaValue tileset_metadata;
    const std::filesystem::path tileset_directory = options.output_directory / "tileset";
    if (options.write_tileset) {
        const CoordinateFrame &frame = preprocessed.read_result.frame;
        RunInfo tileset_run = run;
        tileset_run.local_frame = true;
        tileset_run.geoorigin_mode = "none";
        tileset_run.has_center = true;
        tileset_run.center_x = frame.origin_ecef.x;
        tileset_run.center_y = frame.origin_ecef.y;
        tileset_run.center_z = frame.origin_ecef.z;
        tileset_run.center_longitude = frame.origin.longitude;
        tileset_run.center_latitude = frame.origin.latitude;
        tileset_run.center_height = frame.origin.elevation;
        tileset_metadata = run_metadata(tileset_run);

        std::error_code tileset_error;
        std::filesystem::create_directories(tileset_directory, tileset_error);
        if (tileset_error) {
            LOG_ERROR("error creating tileset directory '" + tileset_directory.string() + "': " + tileset_error.message());
            return 1;
        }
    }
    const auto add_tileset_tile = [&](const Mesh &local_mesh, int i, int j) {
        TilesetTile tile;
        if (!mesh_bounds(local_mesh, tile.minimum, tile.maximum)) {
            return;
        }
        tile.uri = "tile_" + std::to_string(i) + "_" + std::to_string(j) + ".glb";
        std::string tile_error;
        GlbContent content{"tileset_tile", "Surface of one tile of the 3D Tiles tileset, in the east-north-up frame at the center, Y-up", true, i, j};
        content.metadata = &tileset_metadata;
        if (!write_mesh_glb_file(tileset_directory / tile.uri, tileset_content_mesh(local_mesh), &tile_error, content)) {
            LOG_WARN(tile_error);
            return;
        }
        tileset_tiles.push_back(tile);
    };

    CollectedPointCloud collected_points = std::move(preprocessed.read_result.points);

    // ====================================================================================================
    // Point Decimation
    // ====================================================================================================

    const std::size_t input_sample_count = collected_points.size();

    CollectedPointCloud decimated_points;

    if (options.decimation.decimate) {

        LOG_DEBUG("[decimation]", "starting: cell_size=" + std::to_string(options.decimation.cell_size));

        decimated_points = point_decimation(std::move(collected_points), options.decimation);

        if (decimated_points.empty()) {
            LOG_ERROR("point decimation produced no samples");
            return 1;
        }
        LOG_DEBUG("[decimation]", "completed: retained " + std::to_string(decimated_points.size()) + " of " +
                        std::to_string(input_sample_count) + " samples");

    } else {
        LOG_DEBUG("[decimation]", "disabled; using " + std::to_string(input_sample_count) + " input samples");
        decimated_points = std::move(collected_points);
    }

    // ====================================================================================================
    // Reconstruction: normal estimation, screened Poisson, marching cubes, support trimming
    // ====================================================================================================

    const ReconstructionOptions reconstruction = reconstruction_options(options);

    Mesh clean_mesh;
    Mesh raw_mesh;
    OrientedPointCloud oriented_points;
    const Stopwatch reconstruction_timer;

    if (!options.use_chunking) {
        ReconstructionResult result = reconstruct_chunk(decimated_points, reconstruction);
        if (result.status != ReconstructionStatus::Ok) {
            LOG_ERROR(reconstruction_status_message(result.status));
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
            LOG_ERROR("error creating tile directory '" + tiles_directory.string() + "': " + filesystem_error.message());
            return 1;
        }

        const Vec3 tile_offset = placement.use_geoorigin ? geodetic_to_ecef(placement.geoorigin) : Vec3(0.0, 0.0, 0.0);
        const CoordinateFrame &tile_frame = preprocessed.read_result.frame;

        const TileCallback on_tile = [&](TileResult &tile) {
            const std::string tile_name = "tile_" + std::to_string(tile.i) + "_" + std::to_string(tile.j) + ".glb";
            std::string tile_error;
            if (!write_mesh_glb_file(tiles_directory / tile_name, output_mesh(tile.result.mesh, tile_frame, tile_offset), &tile_error,
                                     GlbContent{"tile", "Surface of one reconstruction tile; the tiles together make mesh.glb", true, tile.i, tile.j})) {
                LOG_WARN(tile_error);
            }

            if (options.write_tileset) {
                add_tileset_tile(tile.result.mesh, tile.i, tile.j);
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
        if (!reconstruct_chunked(decimated_points, reconstruction, options.chunking, on_tile, &chunking_stats, &error)) {
            LOG_ERROR(error);
            return 1;
        }

        LOG_INFO("[chunking]", "completed: " + std::to_string(chunking_stats.tiles_processed) + " tiles written, " +
                        std::to_string(chunking_stats.tiles_failed) + " failed, " +
                        std::to_string(chunking_stats.tiles_skipped_sparse) + " skipped as sparse, " +
                        std::to_string(chunking_stats.tiles_empty_after_crop) + " empty");

        if (clean_mesh.vertices.empty() || clean_mesh.indices.empty()) {
            LOG_ERROR("chunked reconstruction produced no mesh; adjust the level of detail or chunk size");
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
                        << ", track_glb=" << (options.write_track_glb ? "enabled" : "disabled")
                        << ", raw_mesh_glb=" << (options.write_raw_mesh_glb ? "enabled" : "disabled");
        LOG_DEBUG("[output]", verbose_message.str());
    }

    PointCloud output_points;
    output_points.reserve(decimated_points.size());
    for (const CollectedPoint &collected_point : decimated_points) {
        output_points.push_back(collected_point.point);
    }

    if (options.write_tileset) {
        if (!options.use_chunking) {
            add_tileset_tile(clean_mesh, 0, 0);   // one tile
        }
        std::string tileset_error;
        if (!write_tileset_json(tileset_directory / "tileset.json", preprocessed.read_result.frame, tileset_tiles, tileset_metadata,
                                "MB-System mbmesh " + mbsystem_version(), &tileset_error)) {
            LOG_ERROR(tileset_error);
            return 1;
        }
        LOG_INFO("[tileset]", "wrote " + (tileset_directory / "tileset.json").string() + " with " + std::to_string(tileset_tiles.size()) + " tiles");
    }

    LOG_INFO("[reconstruction]", "completed in " + reconstruction_timer.text() + (options.use_chunking ? " on " + std::to_string(options.chunking.threads) + " threads" : std::string(", single pass")));

    if (!write_outputs(clean_mesh, raw_mesh, output_points, oriented_points, decimated_points, preprocessed.read_result.frame, placement, preprocessed.read_result.track, options, &error)) {
        LOG_ERROR(error);
        return 1;
    }

    LOG_DEBUG("[output]", "completed");
    LOG_INFO("[complete]", "wrote " + (options.output_directory / "mesh.glb").string() + ", total " + run_timer.text());

    // ====================================================================================================
    // X3DOM Viewer
    // ====================================================================================================

    if (options.write_html) {
        // Written last so that the processing log in the page holds the whole run.
        // The page is in the parent of the output directory, so that the parent is the
        // root of the web server and GLBs beside the output directory can be shown too.
        const std::filesystem::path web_root = viewer_web_root(options.output_directory);
        const std::filesystem::path html_path = viewer_page_path(options.output_directory);

        X3DomWriterOptions html_options;
        html_options.title = viewer_title(web_root);
        html_options.use_geoorigin = placement.use_geoorigin;
        html_options.geo = placement.view;
        html_options.lift_meters = options.lift_meters;
        for (const std::string &text : options.geoviewpoints) {
            X3DomUserViewpoint viewpoint;
            std::string viewpoint_error;
            if (!parse_user_viewpoint(text, &viewpoint, &viewpoint_error)) {
                LOG_ERROR(viewpoint_error);
                return 1;
            }
            html_options.user_viewpoints.push_back(viewpoint);
        }
        html_options.metadata = run_metadata(run);
        html_options.geo_summary = geo_metadata_summary(run);
        html_options.models = find_viewer_models(web_root, options.output_directory, options.write_tileset);
        if (!options.x3dom_url.empty()) {
            html_options.x3dom_url = options.x3dom_url;
        }
        // The log goes in its own file, which the pages load, so that they stay small and can be edited by hand.
        const std::filesystem::path log_path = viewer_log_path(options.output_directory);
        if (!write_log_file(log_path, Logger::captured_messages(), Logger::captured_omitted(), &error)) {
            LOG_ERROR(error);
            return 1;
        }
        html_options.log_url = viewer_log_url(options.output_directory);
        if (!write_glb_x3dom_file(html_path, html_options, &error)) {
            LOG_ERROR(error);
            return 1;
        }
        LOG_INFO("[html]", "wrote " + html_path.string() + ", showing " + std::to_string(html_options.models.size()) + " GLB files");

        // The same page inside the output directory, for a server that is rooted higher up, as one
        // rooted at a survey directory is: it shows the GLBs of this run and no others.
        {
            X3DomWriterOptions inner_options = html_options;
            inner_options.title = viewer_title(web_root);
            inner_options.log_url = log_path.filename().string();
            inner_options.models = find_viewer_models(web_root, options.output_directory, options.write_tileset, true);
            const std::filesystem::path inner_path = viewer_inner_page_path(options.output_directory);
            if (!write_glb_x3dom_file(inner_path, inner_options, &error)) {
                LOG_ERROR(error);
                return 1;
            }
            LOG_INFO("[html]", "wrote " + inner_path.string() + ", showing " + std::to_string(inner_options.models.size()) + " GLB files");
        }

        print_html_viewer_command(web_root, html_path.filename().string());
    }

    return 0;
}

// ========================================================================================================

Options parse_options(int argc, char **argv) {

    Options options;

    // No arguments at all: show the usage, as mbgrd2gltf does, rather than an error about a missing input.
    if (argc < 2) {
        options.help_requested = true;
        return options;
    }

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
        {"-track-glb", "--track-glb"},
        {"-tileset", "--tileset"},
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

    // --metadata was renamed --info. Say so, rather than ignoring an unknown
    // option and starting a full reconstruction, which can run for hours.
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--metadata") == 0 || std::strcmp(argv[i], "-metadata") == 0) {
            LOG_ERROR("the --metadata option has been renamed --info");
            std::exit(1);
        }
    }

    static const struct option long_options[] = {
        {"input", required_argument, nullptr, 'I'},
        {"output", required_argument, nullptr, 'O'},
        {"bounds", required_argument, nullptr, 'R'},
        {"lod", required_argument, nullptr, 'L'},
        {"level-of-detail", required_argument, nullptr, 'L'},
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
        {"track-glb", no_argument, nullptr, 1017},
        {"tileset", no_argument, nullptr, 1019},
        {"x3dom-url", required_argument, nullptr, 1020},
        {"poisson-solver", required_argument, nullptr, 1021},
        {"poisson-iterations", required_argument, nullptr, 1022},
        {"geoviewpoint", required_argument, nullptr, 1018},
        {"lifted", required_argument, nullptr, 1023},
        {"pointcloud-colormap", required_argument, nullptr, 1024},
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
                LOG_WARN("invalid bounds argument: " + std::string(optarg));
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
                    LOG_ERROR("GeoOrigin format must be lon,lat,elev");
                    std::exit(1);
                }
                options.geoorigin.longitude = std::atof(geoorigin_string.substr(0, comma1).c_str());
                options.geoorigin.latitude = std::atof(geoorigin_string.substr(comma1 + 1, comma2 - comma1 - 1).c_str());
                options.geoorigin.elevation = std::atof(geoorigin_string.substr(comma2 + 1).c_str());
                options.geoorigin_mode = GeoOriginMode::Specified;
            }
            break;

        case 1018: {
            X3DomUserViewpoint parsed;
            std::string viewpoint_error;
            if (!parse_user_viewpoint(optarg, &parsed, &viewpoint_error)) {
                LOG_ERROR("invalid --geoviewpoint:", viewpoint_error);
                std::exit(1);
            }
            options.geoviewpoints.push_back(optarg);
            break;
        }

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
                LOG_ERROR("invalid --chunk argument (expected I,J): " + std::string(optarg));
                std::exit(1);
            }
            break;
        }

        case 1015:
            options.max_chunks = static_cast<std::size_t>(std::strtoull(optarg, nullptr, 10));
            break;

        case 1017:
            options.write_track_glb = true;
            break;

        case 1019:
            options.write_tileset = true;
            break;

        case 1020:
            options.x3dom_url = optarg;
            break;

        case 1024:
            if (!parse_point_color_map(optarg, &options.pointcloud_colormap)) {
                LOG_ERROR("invalid --pointcloud-colormap:", std::string("'") + optarg + "' is not one of " + point_color_map_names());
                std::exit(1);
            }
            break;

        case 1023: {
            char *end = nullptr;
            options.lift_meters = std::strtod(optarg, &end);
            if (end == optarg || *end != '\0' || !std::isfinite(options.lift_meters) || options.lift_meters <= 0.0) {
                LOG_ERROR("invalid --lifted:", std::string("'") + optarg + "' is not a number of meters above zero");
                std::exit(1);
            }
            break;
        }

        case 1021:
            if (!parse_poisson_solver(optarg, &options.poisson_solver)) {
                LOG_ERROR("invalid --poisson-solver:", std::string("'") + optarg + "' is not one of " + poisson_solver_names());
                std::exit(1);
            }
            break;

        case 1022: {
            char *end = nullptr;
            const long iterations = std::strtol(optarg, &end, 10);
            if (end == optarg || *end != '\0' || iterations < 1 || iterations > 100000) {
                LOG_ERROR("invalid --poisson-iterations:", std::string("'") + optarg + "' is not a whole number from 1 to 100000");
                std::exit(1);
            }
            options.poisson_iterations = static_cast<int>(iterations);
            break;
        }

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
            options.write_track_glb = true;
            options.write_raw_mesh_glb = true;
            options.write_tileset = true;
            break;

        case 'H':
        case 'h':
            options.help_requested = true;
            break;

        default:
            // getopt has already named the option. An unknown option must not be
            // ignored: it could be a mistyped option that changes the result.
            LOG_ERROR("unrecognized or incomplete option; use -H for usage");
            std::exit(1);
        }
    }

    if (options.input_datalist.empty() && optind < argc) {
        options.input_datalist = argv[optind++];
    }

    // A directory input writes the survey page, which takes --geoviewpoint without --html.
    std::error_code input_error;
    const bool directory_input = !options.input_datalist.empty() && std::filesystem::is_directory(options.input_datalist, input_error);
    if (!options.geoviewpoints.empty() && !options.write_html && !directory_input) {
        LOG_ERROR("--geoviewpoint adds a GeoViewpoint to the page that --html writes; use --html too");
        std::exit(1);
    }
    if (options.lift_meters != 0.0 && !options.write_html && !directory_input) {
        LOG_ERROR("--lifted lifts what the page that --html writes shows; use --html too");
        std::exit(1);
    }

    // Without -O the output goes beside the datalist, so that a run from a survey directory with
    // "-I multibeam/datalistp.mb-1" writes multibeam/mbmesh_output.
    if (options.output_directory.empty()) {
        options.output_directory = options.input_datalist.parent_path() / "mbmesh_output";
    }

    // Anything left over was not understood. Ignoring it can silently change the
    // result: "--geoorigin 122.4,36.6,0" is --geoorigin with no value (the
    // automatic GeoOrigin) followed by a stray argument.
    if (optind < argc) {
        LOG_ERROR("unexpected argument", argv[optind],
                  "- an option value must be attached to its option, for example --geoorigin=lon,lat,elev or -Glon,lat,elev; use -H for usage");
        std::exit(1);
    }

    return options;
}

void print_usage() {
    std::cout << "mbmesh generates 3D meshes from swath sonar bathymetry data.\n\n";
    std::cout << "mbmesh reads the soundings of an MB-System datalist, estimates normals from the neighbors of each\n";
    std::cout << "sounding, fits a surface to them with screened Poisson reconstruction, extracts it with marching\n";
    std::cout << "cubes and trims what the soundings do not support. The output is a glTF binary (GLB), mesh.glb, and,\n";
    std::cout << "optionally, an OGC 3D Tiles tileset and an X3DOM viewer page.\n\n";
    std::cout << "The vertices are in an Earth-Centered, Earth-Fixed (ECEF) Cartesian coordinate system, WGS 84, in\n";
    std::cout << "meters. ECEF is right-handed with its origin at the Earth's center of mass. A GeoOrigin (--geoorigin)\n";
    std::cout << "makes the vertices offsets from a point, which keeps the detail of a localized area in the 32-bit\n";
    std::cout << "floats of a GLB. Heights are above the ellipsoid, with no geoid correction.\n\n";
    std::cout << "usage: mbmesh -I datalist [-R west/east/south/north] [-O outputdir] [-L meters] [output options] [-V]\n\n";
    std::cout << "Required:\n";
    std::cout << "  -I, --input <datalist>       Input MB-System datalist file. A directory writes only a survey page\n";
    std::cout << "                               (mbmesh_survey.html) instead, see below\n\n";
    std::cout << "Optional:\n";
    std::cout << "  -O, --output <outputdir>     Output directory [mbmesh_output in the directory of the datalist]\n";
    std::cout << "  -R, --bounds <w/e/s/n>       Geographic bounds in degrees\n";
    std::cout << "  -L, --lod <meters>           Requested smallest feature size; auto if omitted\n";
    std::cout << "      --decimate <meters>      Enable voxel-grid point decimation\n";
    std::cout << "  -G, --geoorigin[=lon,lat,elev]\n";
    std::cout << "                               Write vertices as ECEF offsets from a GeoOrigin, as in mbgrd2gltf\n";
    std::cout << "                                 With values: use the specified lon,lat,elev\n";
    std::cout << "                                 Without values: use the data center and mean elevation\n";
    std::cout << "                                 Not specified: absolute ECEF coordinates (default)\n";
    std::cout << "                               GLB positions are 32-bit floats, so absolute ECEF (magnitudes of about\n";
    std::cout << "                               6.4 million meters) resolves only about 0.5 m; use a GeoOrigin to\n";
    std::cout << "                               preserve sub-meter detail\n";
    std::cout << "                               The value must be attached to the option, with an equals sign and no\n";
    std::cout << "                               spaces: --geoorigin=-122.4,36.6,0 or -G-122.4,36.6,0. A value in a\n";
    std::cout << "                               separate argument is not accepted\n";
    std::cout << "      --geoviewpoint=<values>  With --html, add a GeoViewpoint to the page, which can be repeated. The values are\n";
    std::cout << "                               geodetic, in the order of --geoorigin, so they do not depend on the GeoOrigin, the\n";
    std::cout << "                               area or the data and can be used again: lon,lat,elev,clon,clat,celev, the camera\n";
    std::cout << "                               position and the point it looks at, or lon,lat,elev,ax,ay,az,angle,clon,clat,celev,\n";
    std::cout << "                               with the orientation as an axis and an angle, as a GeoViewpoint takes it. Heights are\n";
    std::cout << "                               above the ellipsoid. The Copy GeoViewpoint button of the page gives the ten-number form.\n";
    std::cout << "                               The value must be attached: --geoviewpoint=-122.4,36.6,-900,...\n";
    std::cout << "      --lifted <meters>        With --html, or with a directory input, lift everything this many meters above the\n";
    std::cout << "                               ellipsoid along the local vertical: each model goes in a GeoTransform, and the\n";
    std::cout << "                               heights of the viewpoints, --geoviewpoint ones too, are raised by the same amount.\n";
    std::cout << "                               For seafloor data, whose negative heights parts of X3DOM's GeoTileset do not handle;\n";
    std::cout << "                               use more than the greatest depth. Copy GeoViewpoint takes the lift off again\n";
    std::cout << "      --chunk-size <meters>    Reconstruct in square tiles of about this size, writing tiles/tile_I_J.glb\n";
    std::cout << "                               plus the merged mesh.glb; chunking turns on automatically for\n";
    std::cout << "                               surveys too large for one Poisson grid\n";
    std::cout << "      --chunk <i,j>            Reconstruct only tile (i,j); tile (i,j) covers x in [i*size,(i+1)*size)\n";
    std::cout << "                               and y in [j*size,(j+1)*size) in the local east-north frame\n";
    std::cout << "      --max-chunks <n>         Stop after n tiles (for testing)\n";
    std::cout << "      --threads <n>            Tiles reconstructed at once when chunking [min(cores,4)]; each holds its own grid\n";
    std::cout << "      --poisson-solver <name>  How the Poisson grid equation is solved: " << poisson_solver_names() << " [sor]\n";
    std::cout << "      --poisson-iterations <n> Iterations of the solver, for either solver [sor: one per cell across the longest grid edge,\n";
    std::cout << "                               at least 50; jacobi: chosen from the level of detail, 200 to 500]. The\n";
    std::cout << "                               relative residual after them is logged with -V, and at INFO in a single pass\n";
    std::cout << "      --info                   Print dataset information and the reconstruction plan, then exit\n";
    std::cout << "  -V, --verbose                Enable progress and diagnostic logging\n";
    std::cout << "  -H, -h, --help               Print this help message\n\n";
    std::cout << "Output options:\n";
    std::cout << "      --html                   Also write an X3DOM viewer page, with a check box for each GLB and the\n";
    std::cout << "                               processing log, in the parent of the output directory (for example\n";
    std::cout << "                               mbmesh_output.html), and print the command to view it. The output directory\n";
    std::cout << "                               gets its own page too, mbmesh.html, without the GLBs beside the directory\n";
    std::cout << "      --xyz                    Also write local and ECEF XYZ point clouds\n";
    std::cout << "      --local-xyz              Also write pointcloud-local.xyz\n";
    std::cout << "      --ecef-xyz               Also write pointcloud-ecef.xyz\n";
    std::cout << "      --oriented-ply           Also write oriented-pointcloud-ecef.ply\n";
    std::cout << "      --pointcloud-glb         Also write pointcloud.glb, colored by height\n";
    std::cout << "      --pointcloud-colormap <name>  How pointcloud.glb is colored by height, with the Haxby color table of mbm_grdplot:\n";
    std::cout << "                               " << point_color_map_names() << " [linear]. linear: 11 equal steps between rounded heights, as\n";
    std::cout << "                               mbm_grdplot -G5 does; equalized: steps with equal numbers of points, as mbm_grdplot -G5 -S;\n";
    std::cout << "                               none: one orange. The heights are listed in the file's description\n";
    std::cout << "      --normal-glb             Also write normals.glb\n";
    std::cout << "      --origin-glb             Also write origins.glb\n";
    std::cout << "      --track-glb              Also write track.glb, the sensor track as a black 3D line\n";
    std::cout << "      --tileset                Also write tileset/tileset.json and tileset/tile_I_J.glb, the surface as an OGC 3D\n";
    std::cout << "                               Tiles tileset (a part of it is loaded as needed, so large surveys fit in a browser).\n";
    std::cout << "                               With --html the page shows it with a GeoTileset node, which needs a build of X3DOM\n";
    std::cout << "                               that has one: see --x3dom-url\n";
    std::cout << "      --x3dom-url <url>        The X3DOM script of the --html page [https://www.x3dom.org/x3dom/dist/x3dom-full.js]\n";
    std::cout << "      --raw-mesh-glb           Also write raw_mesh.glb before support trimming\n";
    std::cout << "      --diagnostics            Also write pointcloud, normal, and origin GLBs\n";
    std::cout << "      --all-outputs            Write all optional outputs\n\n";
    std::cout << "The default output is mesh.glb.\n\n";
    std::cout << "Survey page: when -I names a directory, for example a survey directory with multibeam and lidar\n";
    std::cout << "subdirectories, mbmesh reconstructs nothing. It writes mbmesh_survey.html in the current directory,\n";
    std::cout << "a viewer page with a check box for every .glb file beneath that directory, in accordions that mirror\n";
    std::cout << "its layout (the tiles of a tileset are left out; its tileset.json is offered; the track.glb files start switched on). Run it where a\n";
    std::cout << "web server will serve the directory, and the page loads the GLBs by relative path. The page\n";
    std::cout << "has an overview viewpoint over all the mbmesh GLBs found, with the GeoOrigin of the first, so GLBs from runs with another GeoOrigin\n";
    std::cout << "do not line up with it; each is logged as an ERROR. --geoviewpoint adds GeoViewpoints to its Viewpoint menu, as it does with --html;\n";
    std::cout << "--x3dom-url sets its X3DOM script.\n";
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
    const std::vector<TrackPoint> &track,
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
        LOG_DEBUG("[output]", "wrote " + ecef_xyz_path.string());
    }

    if (options.write_local_xyz) {
        const auto local_xyz_path = options.output_directory / "pointcloud-local.xyz";
        if (!write_local_xyz_pointcloud(points, local_xyz_path.string(), error)) {
            return false;
        }
        LOG_DEBUG("[output]", "wrote " + local_xyz_path.string());
    }

    if (options.write_oriented_ply) {
        const auto oriented_ply_path = options.output_directory / "oriented-pointcloud-ecef.ply";
        if (!write_ply_oriented_pointcloud(oriented_points, frame, oriented_ply_path.string(), error)) {
            return false;
        }
        LOG_DEBUG("[output]", "wrote " + oriented_ply_path.string());
    }

    if (options.write_pointcloud_glb) {
        const auto pointcloud_glb_path = options.output_directory / "pointcloud.glb";
        // The points are colored by their height in the local frame, which is up and in meters.
        std::vector<double> heights;
        heights.reserve(points.size());
        for (const Vec3 &point : points) {
            heights.push_back(point.z);
        }
        if (!write_pointcloud_glb_file(pointcloud_glb_path, output_points(points, frame, offset), error,
                                         GlbContent{"pointcloud", "The soundings used for the reconstruction, as points"}, &heights,
                                       options.pointcloud_colormap)) {
            return false;
        }
        LOG_DEBUG("[output]", "wrote " + pointcloud_glb_path.string());
    }

    if (options.write_normal_glb) {
        const auto normals_glb_path = options.output_directory / "normals.glb";
        if (!write_normal_lines_glb_file(normals_glb_path, output_oriented_points(oriented_points, frame, offset), 0.25, error,
                                         GlbContent{"normals", "The estimated surface normal at each sounding, as short lines"})) {
            return false;
        }
        LOG_DEBUG("[output]", "wrote " + normals_glb_path.string());
    }

    if (options.write_origin_glb) {
        const auto origins_glb_path = options.output_directory / "origins.glb";
        if (!write_origin_ray_lines_glb_file(origins_glb_path, output_collected_points(collected_points, frame, offset), 0.25, error,
                                         GlbContent{"origin_rays", "A short line from each sounding toward the sensor that measured it"})) {
            return false;
        }
        LOG_DEBUG("[output]", "wrote " + origins_glb_path.string());
    }

    if (options.write_track_glb) {
        const auto track_glb_path = options.output_directory / "track.glb";
        // The track must be in exactly the coordinates of mesh.glb, so that the two
        // overlay: it goes through the same output_position(), with the same frame
        // and the same GeoOrigin offset (computed once, above), as every mesh.
        std::vector<std::vector<Vec3>> polylines = split_track(track);
        std::size_t track_points = 0;
        for (std::vector<Vec3> &polyline : polylines) {
            track_points += polyline.size();
            for (Vec3 &position : polyline) {
                position = output_position(position, frame, offset);
            }
        }
        if (!write_track_glb_file(track_glb_path, polylines, error,
                                  GlbContent{"track", "The sensor track, one line per continuous run of pings, in the same coordinates as mesh.glb"})) {
            return false;
        }
        LOG_DEBUG("[output]", "wrote " + track_glb_path.string() + ": " + std::to_string(track.size()) + " pings in " +
                                  std::to_string(polylines.size()) + " polylines, " +
                                  std::to_string(track_points) + " points drawn");
    }

    if (options.write_raw_mesh_glb) {
        const auto raw_mesh_glb_path = options.output_directory / "raw_mesh.glb";
        if (!write_mesh_glb_file(raw_mesh_glb_path, output_mesh(raw_mesh, frame, offset), error,
                                  GlbContent{"raw_mesh", "The reconstructed surface before support trimming"})) {
            return false;
        }
        LOG_DEBUG("[output]", "wrote " + raw_mesh_glb_path.string());
    }

    const auto clean_mesh_glb_path = options.output_directory / "mesh.glb";
    if (!write_mesh_glb_file(clean_mesh_glb_path, output_mesh(mesh, frame, offset), error,
                              GlbContent{"mesh", "The reconstructed seafloor surface"})) {
        return false;
    }
    LOG_DEBUG("[output]", "wrote " + clean_mesh_glb_path.string());

    return true;
}

// ========================================================================================================

void print_html_viewer_command(
    const std::filesystem::path &directory,
    const std::string &html_filename)
{
    // The viewer page loads its GLBs over HTTP, which browsers require for local
    // files. The directory is the web root, the parent of the output directory. Rather than starting a server and a browser, which would leave a
    // process running and hold the port, log the command to run.
    constexpr int port = 8000;

    std::error_code filesystem_error;
    std::filesystem::path absolute_directory = std::filesystem::absolute(directory, filesystem_error);
    if (filesystem_error) {
        absolute_directory = directory;
    }

    LOG_INFO("[html]", "to view " + html_filename + ", run this command in a terminal (Ctrl-C stops it), then open the URL:");
    LOG_INFO("[html]", "  python3 -m http.server " + std::to_string(port) + " --bind 127.0.0.1 --directory " +
                    shell_quote(absolute_directory.string()));
    LOG_INFO("[html]", "  http://127.0.0.1:" + std::to_string(port) + "/" + html_filename);
}
