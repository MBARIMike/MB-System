#include "settings.h"

#include "util/logger.h"

#include "math/kdtree.h"

#include <algorithm>
#include <cmath>
#include <thread>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <vector>

namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double minimum_feature_size = 0.01; // meters
constexpr double maximum_feature_size = 5.0;  // meters
constexpr double automatic_feature_size_spacing_scale = 2.0;
constexpr double maximum_poisson_grid_cells = 45.0e6;

// Target size of one chunk's Poisson grid. The solver holds seven grids of
// about 72 bytes per cell in all, so this is roughly 0.6 GB.
constexpr double target_chunk_grid_cells = 8.0e6;
constexpr double minimum_auto_chunk_cells = 16.0;
constexpr double maximum_auto_chunk_cells = 512.0;
constexpr std::size_t maximum_spacing_samples = 8192;

struct PoissonGridEstimate {
    std::size_t nx = 0;
    std::size_t ny = 0;
    std::size_t nz = 0;
    double cell_count = 0.0;
};

void apply_cell_size_dependent_defaults(Options &options, double cell_size, double average_spacing);

Bounds3D collected_point_bounds(const CollectedPointCloud &points) {
    Bounds3D bounds(points.front().point, points.front().point);
    for (const CollectedPoint &point : points) {
        bounds.include(point.point);
    }
    return bounds;
}

GeodeticBounds collected_geodetic_bounds(const CollectedPointCloud &points, const CoordinateFrame &frame, double *mean_elevation) {
    GeodeticBounds bounds;
    bool initialized = false;
    double elevation_sum = 0.0;
    for (const CollectedPoint &point : points) {
        const GeodeticPosition position = local_to_geodetic(point.point, frame);
        elevation_sum += position.elevation;

        if (!initialized) {
            bounds.min_lon = position.longitude;
            bounds.max_lon = position.longitude;
            bounds.min_lat = position.latitude;
            bounds.max_lat = position.latitude;
            initialized = true;
            continue;
        }

        bounds.min_lon = std::min(bounds.min_lon, position.longitude);
        bounds.max_lon = std::max(bounds.max_lon, position.longitude);
        bounds.min_lat = std::min(bounds.min_lat, position.latitude);
        bounds.max_lat = std::max(bounds.max_lat, position.latitude);
    }
    if (mean_elevation != nullptr && !points.empty()) {
        *mean_elevation = elevation_sum / static_cast<double>(points.size());
    }
    return bounds;
}

std::vector<Vec3> sampled_positions(const CollectedPointCloud &points, std::size_t maximum_samples) {
    std::vector<Vec3> positions;
    if (points.empty() || maximum_samples == 0) {
        return positions;
    }

    const std::size_t sample_count = std::min(points.size(), maximum_samples);
    positions.reserve(sample_count);
    for (std::size_t i = 0; i < sample_count; i++) {
        const std::size_t point_index =
            sample_count == 1 ? 0 : i * (points.size() - 1) / (sample_count - 1);
        positions.push_back(points[point_index].point);
    }
    return positions;
}

SpacingMetadata estimate_point_spacing(const CollectedPointCloud &points) {
    SpacingMetadata metadata;

    const std::vector<Vec3> positions = sampled_positions(points, maximum_spacing_samples);
    metadata.sample_count = positions.size();
    if (positions.size() < 2) {
        return metadata;
    }

    const KDTree tree(positions);
    std::vector<double> distances;
    distances.reserve(positions.size());

    for (std::size_t i = 0; i < positions.size(); i++) {
        const std::vector<KDTree::Neighbor> neighbors = tree.k_nearest(positions[i], 1, i);
        if (neighbors.empty()) {
            continue;
        }

        const double distance_squared = neighbors.front().distance_squared;
        if (distance_squared > vec3_epsilon * vec3_epsilon) {
            distances.push_back(std::sqrt(distance_squared));
        }
    }

    if (distances.empty()) {
        return metadata;
    }

    std::sort(distances.begin(), distances.end());
    const double distance_sum = std::accumulate(distances.begin(), distances.end(), 0.0);

    metadata.minimum = distances.front();
    metadata.maximum = distances.back();
    metadata.average = distance_sum / static_cast<double>(distances.size());
    metadata.median = distances[distances.size() / 2];
    return metadata;
}

PoissonGridEstimate estimate_poisson_grid(const Bounds3D &bounds, double cell_size, double padding) {
    PoissonGridEstimate estimate;
    if (cell_size <= 0.0 || !std::isfinite(cell_size) || !std::isfinite(padding)) {
        return estimate;
    }

    const Vec3 size = bounds.size() + Vec3(2.0 * padding);
    estimate.nx = static_cast<std::size_t>(std::ceil(std::max(0.0, size.x) / cell_size)) + 1;
    estimate.ny = static_cast<std::size_t>(std::ceil(std::max(0.0, size.y) / cell_size)) + 1;
    estimate.nz = static_cast<std::size_t>(std::ceil(std::max(0.0, size.z) / cell_size)) + 1;
    estimate.cell_count = static_cast<double>(estimate.nx) *
                          static_cast<double>(estimate.ny) *
                          static_cast<double>(estimate.nz);
    return estimate;
}

void apply_cell_size_dependent_defaults(Options &options, double cell_size, double average_spacing) {
    const double resolved_cell_size = std::max(cell_size, minimum_feature_size * 0.05);

    options.poisson.cell_size = resolved_cell_size;
    options.poisson.padding = std::clamp(4.0 * resolved_cell_size, 2.0 * resolved_cell_size, 5.0);
    options.poisson.normal_splat_radius = std::max(2.0 * resolved_cell_size, 1.5 * average_spacing);

    options.trimming.support_radius = std::max(4.0 * resolved_cell_size, 2.5 * average_spacing);
    options.trimming.max_normal_offset = std::max(2.0 * resolved_cell_size, 1.25 * average_spacing);
}

void print_warnings(const Options &options, const DatalistMetadata &metadata) {
    if (!metadata.lod_supported_by_spacing) {
        LOG_WARN("resolved level_of_detail", options.level_of_detail,
                 "m is not supported by the estimated average point spacing", metadata.spacing.average,
                 "m; the result may be inaccurate");
    }

    if (metadata.normal_radius_exceeds_feature_limit) {
        LOG_WARN("normal estimation radius", options.normals.search_radius,
                 "m exceeds 45% of requested feature size", options.level_of_detail,
                 "m; normals may smooth features smaller than the radius");
    }
}

} // namespace

void apply_spacing_driven_defaults(Options &options, DatalistMetadata *metadata) {
    const double fallback_feature_size =
        options.level_of_detail > vec3_epsilon ? options.level_of_detail : minimum_feature_size;
    const double average_spacing =
        metadata->spacing.average > vec3_epsilon ? metadata->spacing.average : fallback_feature_size;
    const double desired_feature_size =
        std::clamp(
            options.level_of_detail_requested
                ? options.level_of_detail
                : automatic_feature_size_spacing_scale * average_spacing,
            minimum_feature_size,
            maximum_feature_size);

    options.level_of_detail = desired_feature_size;

    if (!options.decimation_requested) {
        options.decimation.decimate = false;
        options.decimation.cell_size = std::max(0.5 * desired_feature_size, 0.5 * average_spacing);
    }

    const double poisson_cell_size =
        std::clamp(desired_feature_size / 4.0, average_spacing / 2.0, average_spacing);
    apply_cell_size_dependent_defaults(options, poisson_cell_size, average_spacing);

    const double normal_radius =
        std::max(2.5 * average_spacing, 2.5 * options.poisson.cell_size);
    metadata->normal_radius_exceeds_feature_limit =
        normal_radius > desired_feature_size;

    options.normals.search_radius = normal_radius;
    options.normals.k = static_cast<std::size_t>(std::clamp(
        std::lround(pi * std::pow(normal_radius / average_spacing, 2.0)),
        24L,
        48L));
    options.normals.minimum_neighbors = 6;

    options.poisson.screening_weight =
        std::clamp(4.0 * options.poisson.cell_size / average_spacing, 1.0, 6.0);
    options.poisson.solver_iterations = std::clamp(
        static_cast<int>(std::round(220.0 + 80.0 / std::sqrt(desired_feature_size))),
        200,
        500);
    options.poisson.use_screening = true;
    options.poisson.iso_value = 0.0;

    options.marching_cubes.iso_value = 0.0;

    options.trimming.enabled = true;
    options.trimming.minimum_neighbors = 3;
    options.trimming.minimum_normal_alignment = 0.0;

    metadata->lod_supported_by_spacing =
        desired_feature_size / average_spacing >= automatic_feature_size_spacing_scale;
}

ReconstructionOptions reconstruction_options(const Options &options) {
    ReconstructionOptions reconstruction;
    reconstruction.normals = options.normals;
    reconstruction.poisson = options.poisson;
    reconstruction.marching_cubes = options.marching_cubes;
    reconstruction.trimming = options.trimming;
    return reconstruction;
}

bool resolve_chunking(Options &options, const DatalistMetadata &metadata, std::string *error) {
    options.use_chunking = false;

    const double cell = options.poisson.cell_size;
    const PoissonGridEstimate whole = estimate_poisson_grid(metadata.bounds, cell, options.poisson.padding);

    const bool forced = options.chunk_size_requested || options.chunk_tile_requested || options.max_chunks > 0;
    if (!forced && whole.cell_count <= target_chunk_grid_cells) {
        return true;   // small enough to reconstruct in one piece
    }

    const ReconstructionOptions reconstruction = reconstruction_options(options);
    const double halo = chunk_halo(reconstruction);

    double tile_size = 0.0;
    if (options.chunk_size_requested) {
        if (!(options.chunk_size > 0.0) || !std::isfinite(options.chunk_size)) {
            if (error != nullptr) {
                *error = "--chunk-size must be a positive number of meters";
            }
            return false;
        }
        tile_size = round_to_cells(options.chunk_size, cell);
    }
    else {
        // Pick the tile edge so a tile's grid, including its halo and padding,
        // holds about target_chunk_grid_cells. The survey's full depth range is
        // used for the vertical extent, which is the worst case for any tile.
        const double depth_cells = std::ceil((metadata.bounds.size().z + 2.0 * options.poisson.padding) / cell) + 1.0;
        const double columns = target_chunk_grid_cells / std::max(1.0, depth_cells);
        const double side_cells = std::floor(std::sqrt(columns));
        const double margin_cells = 2.0 * (halo + options.poisson.padding) / cell;
        const double tile_cells = std::clamp(side_cells - margin_cells, minimum_auto_chunk_cells, maximum_auto_chunk_cells);
        tile_size = tile_cells * cell;
    }

    options.use_chunking = true;
    options.chunking.tile_size = tile_size;
    options.chunking.halo = halo;
    options.chunking.single_tile = options.chunk_tile_requested;
    options.chunking.tile_i = options.chunk_tile_i;
    options.chunking.tile_j = options.chunk_tile_j;
    options.chunking.max_tiles = options.max_chunks;

    // Each tile holds its own Poisson grid, so cap the automatic thread count to
    // keep memory modest. Ask for more with --threads.
    constexpr std::size_t automatic_thread_limit = 4;
    const std::size_t hardware = std::max<std::size_t>(1, std::thread::hardware_concurrency());
    options.chunking.threads = options.threads > 0 ? options.threads : std::min(hardware, automatic_thread_limit);
    return true;
}

bool resolve_geoorigin(const Options &options, const DatalistMetadata &metadata, GeodeticPosition *origin) {
    if (options.geoorigin_mode == GeoOriginMode::Specified) {
        *origin = options.geoorigin;
        return true;
    }
    if (options.geoorigin_mode == GeoOriginMode::Auto) {
        // Same convention as mbgrd2gltf: center of the bounds, mean elevation.
        origin->longitude = (metadata.geodetic_bounds.min_lon + metadata.geodetic_bounds.max_lon) / 2.0;
        origin->latitude = (metadata.geodetic_bounds.min_lat + metadata.geodetic_bounds.max_lat) / 2.0;
        origin->elevation = metadata.mean_elevation;
        return true;
    }
    return false;
}

bool adjust_poisson_grid_to_budget(Options &options, const DatalistMetadata &metadata, std::string *error) {
    PoissonGridEstimate estimate =
        estimate_poisson_grid(metadata.bounds, options.poisson.cell_size, options.poisson.padding);
    if (estimate.cell_count <= maximum_poisson_grid_cells) {
        return true;
    }

    if (error != nullptr) {
        std::ostringstream message;
        message << "Poisson grid estimate " << estimate.cell_count << " cells exceeds the current budget "
                << maximum_poisson_grid_cells << " at cell size " << options.poisson.cell_size
                << " m; reduce the requested LOD or run the reconstruction with chunked/adaptive processing";
        *error = message.str();
    }
    return false;
}

bool preprocess_datalist(Options &options, PreprocessedDatalist *preprocessed, std::string *error) {
    if (preprocessed == nullptr) {
        if (error != nullptr) {
            *error = "Preprocessed datalist result pointer is null";
        }
        return false;
    }

    DatalistReaderOptions reader_options;
    reader_options.use_bounds = options.use_bounds;
    reader_options.bounds = options.bounds;
    reader_options.verbose = options.verbose ? 1 : 0;

    if (!read_datalist(options.input_datalist, reader_options, &preprocessed->read_result, error)) {
        return false;
    }

    if (preprocessed->read_result.points.empty()) {
        if (error != nullptr) {
            *error = "input contains no accepted soundings";
        }
        return false;
    }

    preprocessed->metadata.point_count = preprocessed->read_result.points.size();
    preprocessed->metadata.bounds = collected_point_bounds(preprocessed->read_result.points);
    preprocessed->metadata.geodetic_bounds = collected_geodetic_bounds(
        preprocessed->read_result.points, preprocessed->read_result.frame, &preprocessed->metadata.mean_elevation);
    preprocessed->metadata.spacing = estimate_point_spacing(preprocessed->read_result.points);

    apply_spacing_driven_defaults(options, &preprocessed->metadata);
    if (!resolve_chunking(options, preprocessed->metadata, error)) {
        return false;
    }
    if (!options.use_chunking && !adjust_poisson_grid_to_budget(options, preprocessed->metadata, error)) {
        return false;
    }
    print_warnings(options, preprocessed->metadata);

    return true;
}

void print_datalist_metadata(const PreprocessedDatalist &preprocessed, const Options &options) {
    const DatalistMetadata &metadata = preprocessed.metadata;
    const DatalistReaderStats &stats = preprocessed.read_result.stats;
    const Bounds3D &bounds = metadata.bounds;
    const Vec3 extent = bounds.size();
    const PoissonGridEstimate grid =
        estimate_poisson_grid(bounds, options.poisson.cell_size, options.poisson.padding);

    std::cout << std::setprecision(10);
    std::cout << "mbmesh dataset metadata\n";
    std::cout << "input:\n";
    std::cout << "  files_read: " << stats.files_read << '\n';
    std::cout << "  soundings_read: " << stats.soundings_read << '\n';
    std::cout << "  accepted_soundings: " << metadata.point_count << '\n';

    std::cout << "geodetic_bounds_degrees:\n";
    std::cout << "  min_lon: " << metadata.geodetic_bounds.min_lon << '\n';
    std::cout << "  max_lon: " << metadata.geodetic_bounds.max_lon << '\n';
    std::cout << "  min_lat: " << metadata.geodetic_bounds.min_lat << '\n';
    std::cout << "  max_lat: " << metadata.geodetic_bounds.max_lat << '\n';
    std::cout << "  extent_lon: " << (metadata.geodetic_bounds.max_lon - metadata.geodetic_bounds.min_lon) << '\n';
    std::cout << "  extent_lat: " << (metadata.geodetic_bounds.max_lat - metadata.geodetic_bounds.min_lat) << '\n';
    std::cout << "  point_count: " << metadata.point_count << '\n';

    std::cout << "estimated_spacing_meters:\n";
    std::cout << "  sample_count: " << metadata.spacing.sample_count << '\n';
    std::cout << "  minimum: " << metadata.spacing.minimum << '\n';
    std::cout << "  maximum: " << metadata.spacing.maximum << '\n';
    std::cout << "  average: " << metadata.spacing.average << '\n';
    std::cout << "  median: " << metadata.spacing.median << '\n';

    std::cout << "reconstruction:\n";
    std::cout << "  poisson_cell_size: " << options.poisson.cell_size << '\n';
    std::cout << "  whole_survey_grid_cells: " << grid.cell_count << '\n';
    std::cout << "  chunked: " << (options.use_chunking ? "yes" : "no") << '\n';
    if (options.use_chunking) {
        const double tile = options.chunking.tile_size;
        const double halo = options.chunking.halo;
        const Vec3 survey = extent;
        const double tiles_x = std::ceil(survey.x / tile) + 1.0;
        const double tiles_y = std::ceil(survey.y / tile) + 1.0;
        const double side = (tile + 2.0 * (halo + options.poisson.padding)) / options.poisson.cell_size + 1.0;
        std::cout << "  tile_size: " << tile << '\n';
        std::cout << "  halo: " << halo << '\n';
        std::cout << "  tiles_at_most: " << tiles_x * tiles_y << '\n';
        std::cout << "  tile_grid_columns: " << side * side << '\n';
    }
}
