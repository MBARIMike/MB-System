#pragma once

#include <cstddef>
#include <filesystem>
#include <string>

#include "algorithms/marching_cubes.h"
#include "algorithms/normal_estimation.h"
#include "algorithms/point_decimation.h"
#include "algorithms/reconstruction.h"
#include "algorithms/screened_poisson.h"
#include "algorithms/support_trimming.h"
#include "data_types/geometry.h"
#include "data_types/swathfile.h"
#include "io/datalist_reader.h"

enum class GeoOriginMode {
    None,       // absolute ECEF vertices (default)
    Auto,       // -G with no value: center of the data bounds and mean elevation
    Specified,  // -G lon,lat,elev
};

struct Options {
    bool help_requested = false;
    bool metadata_requested = false;
    bool verbose = false;

    bool write_html = false;
    bool write_local_xyz = false;
    bool write_ecef_xyz = false;
    bool write_oriented_ply = false;
    bool write_pointcloud_glb = false;
    bool write_normal_glb = false;
    bool write_origin_glb = false;
    bool write_raw_mesh_glb = false;

    std::filesystem::path input_datalist;
    std::filesystem::path output_directory = "mbmesh_output";

    bool use_bounds = false;
    GeographicBounds bounds;

    // --geoorigin / -G, with the same meaning as in mbgrd2gltf. Output
    // vertices are WGS84 ECEF coordinates; with a GeoOrigin they are offsets
    // from that point's ECEF position instead.
    GeoOriginMode geoorigin_mode = GeoOriginMode::None;
    GeodeticPosition geoorigin;   // used when geoorigin_mode == Specified

    // Smallest feature size, in local meters, that mbmesh should try to preserve.
    double level_of_detail = 0.5;
    bool level_of_detail_requested = false;
    bool decimation_requested = false;

    // Chunked reconstruction. --chunk-size forces it on with the given tile size
    // in meters; otherwise it turns on automatically when a single Poisson grid
    // would be too large. chunking and use_chunking are resolved from these in
    // preprocess_datalist().
    bool chunk_size_requested = false;
    double chunk_size = 0.0;
    bool chunk_tile_requested = false;
    int chunk_tile_i = 0;
    int chunk_tile_j = 0;
    std::size_t max_chunks = 0;
    std::size_t threads = 0;   // tiles reconstructed at once; 0 chooses automatically
    bool use_chunking = false;
    ChunkingOptions chunking;

    PointDecimationOptions decimation;
    NormalEstimationOptions normals;
    ScreenedPoissonOptions poisson;
    MarchingCubesOptions marching_cubes;
    SupportTrimmingOptions trimming;
};

struct SpacingMetadata {
    double minimum = 0.0;
    double maximum = 0.0;
    double average = 0.0;
    double median = 0.0;
    std::size_t sample_count = 0;
};

struct GeodeticBounds {
    double min_lon = 0.0;
    double max_lon = 0.0;
    double min_lat = 0.0;
    double max_lat = 0.0;
};

struct DatalistMetadata {
    Bounds3D bounds;
    GeodeticBounds geodetic_bounds;
    SpacingMetadata spacing;
    double mean_elevation = 0.0;   // mean accepted sounding elevation, meters above the ellipsoid
    std::size_t point_count = 0;
    bool lod_supported_by_spacing = true;
    bool normal_radius_exceeds_feature_limit = false;
};

struct PreprocessedDatalist {
    DatalistReadResult read_result;
    DatalistMetadata metadata;
};

// Resolves the GeoOrigin that --geoorigin selects. Returns false (and leaves
// *origin untouched) when no GeoOrigin was requested.
bool resolve_geoorigin(const Options &options, const DatalistMetadata &metadata, GeodeticPosition *origin);

// Builds the options the reconstruction stages use.
ReconstructionOptions reconstruction_options(const Options &options);

// Decides whether to chunk and, if so, sets options.use_chunking and
// options.chunking. Returns false (with *error) if the settings cannot work.
// If points is given, the tile count is taken from the tiles that contain
// points; otherwise from the survey's bounding box.
bool resolve_chunking(Options &options, const DatalistMetadata &metadata, const CollectedPointCloud *points = nullptr, std::string *error = nullptr);

void apply_spacing_driven_defaults(Options &options, DatalistMetadata *metadata);
bool adjust_poisson_grid_to_budget(Options &options, const DatalistMetadata &metadata, std::string *error = nullptr);

bool preprocess_datalist(
    Options &options,
    PreprocessedDatalist *preprocessed,
    std::string *error);

void print_datalist_metadata(
    const PreprocessedDatalist &preprocessed,
    const Options &options);
