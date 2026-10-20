#include "settings.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace {

bool nearly_equal(double a, double b, double tolerance = 1.0e-9) {
    return std::fabs(a - b) <= tolerance;
}

bool check(bool condition, const char *message) {
    if (!condition) {
        std::fprintf(stderr, "%s\n", message);
        return false;
    }
    return true;
}

} // namespace

int main() {
    DatalistMetadata metadata;
    metadata.spacing.average = 0.20;
    metadata.bounds = Bounds3D(Vec3(0.0, 0.0, -1.0), Vec3(5.0, 3.0, 1.0));

    Options automatic_options;
    apply_spacing_driven_defaults(automatic_options, &metadata);

    if (!check(nearly_equal(automatic_options.level_of_detail, 0.40),
               "Automatic LOD should be twice the estimated average point spacing")) {
        return 1;
    }
    if (!check(!automatic_options.decimation.decimate,
               "Spacing-driven defaults should leave decimation disabled unless requested")) {
        return 1;
    }
    if (!check(automatic_options.normals.search_radius > 0.0,
               "Normal-estimation radius should be initialized")) {
        return 1;
    }
    if (!check(automatic_options.poisson.cell_size > 0.0,
               "Poisson cell size should be initialized")) {
        return 1;
    }
    if (!check(automatic_options.trimming.enabled,
               "Support trimming should be enabled by default")) {
        return 1;
    }

    Options requested_options;
    requested_options.level_of_detail_requested = true;
    requested_options.level_of_detail = 0.80;
    requested_options.decimation_requested = true;
    requested_options.decimation.decimate = true;
    requested_options.decimation.cell_size = 0.25;

    DatalistMetadata requested_metadata;
    requested_metadata.spacing.average = 0.20;
    requested_metadata.bounds = metadata.bounds;
    apply_spacing_driven_defaults(requested_options, &requested_metadata);

    if (!check(nearly_equal(requested_options.level_of_detail, 0.80),
               "Explicitly requested LOD should be preserved within the supported range")) {
        return 1;
    }
    if (!check(requested_options.decimation.decimate &&
                   nearly_equal(requested_options.decimation.cell_size, 0.25),
               "Requested decimation settings should be preserved")) {
        return 1;
    }
    if (!check(requested_metadata.lod_supported_by_spacing,
               "Requested LOD should be marked supported when it is large enough for point spacing")) {
        return 1;
    }

    std::string error;
    if (!check(adjust_poisson_grid_to_budget(requested_options, requested_metadata, &error),
               "Small test bounds should fit within the Poisson grid budget")) {
        return 1;
    }

    Options oversized_options = requested_options;
    oversized_options.poisson.cell_size = 0.01;
    oversized_options.poisson.padding = 0.0;

    DatalistMetadata oversized_metadata = requested_metadata;
    oversized_metadata.bounds = Bounds3D(Vec3(0.0), Vec3(1000.0));

    error.clear();
    if (!check(!adjust_poisson_grid_to_budget(oversized_options, oversized_metadata, &error),
               "Oversized grid should be rejected by the Poisson grid budget")) {
        return 1;
    }
    if (!check(!error.empty(), "Grid-budget rejection should explain the failure")) {
        return 1;
    }

    // --geoorigin resolution follows mbgrd2gltf: none by default, the given
    // lon,lat,elev when specified, otherwise the bounds center and mean elevation.
    DatalistMetadata geo_metadata;
    geo_metadata.geodetic_bounds = {-122.4, -122.2, 36.5, 36.7};
    geo_metadata.mean_elevation = -1500.0;

    Options geo_options;
    GeodeticPosition geo_origin = {1.0, 2.0, 3.0};
    if (!check(!resolve_geoorigin(geo_options, geo_metadata, &geo_origin) && geo_origin.longitude == 1.0,
               "No --geoorigin should leave absolute ECEF output and the origin untouched")) {
        return 1;
    }

    geo_options.geoorigin_mode = GeoOriginMode::Auto;
    if (!check(resolve_geoorigin(geo_options, geo_metadata, &geo_origin) &&
                   nearly_equal(geo_origin.longitude, -122.3) && nearly_equal(geo_origin.latitude, 36.6) &&
                   nearly_equal(geo_origin.elevation, -1500.0),
               "Automatic GeoOrigin should be the bounds center at the mean elevation")) {
        return 1;
    }

    geo_options.geoorigin_mode = GeoOriginMode::Specified;
    geo_options.geoorigin = {-121.9, 36.8, -10.0};
    if (!check(resolve_geoorigin(geo_options, geo_metadata, &geo_origin) &&
                   nearly_equal(geo_origin.longitude, -121.9) && nearly_equal(geo_origin.latitude, 36.8) &&
                   nearly_equal(geo_origin.elevation, -10.0),
               "Specified GeoOrigin should be used as given")) {
        return 1;
    }

    // Automatic tiling must produce enough tiles to keep every thread busy. A
    // coarse-celled survey with a modest depth range would otherwise get a few
    // huge tiles, so that extra threads have nothing to do.
    {
        DatalistMetadata big;
        big.spacing.average = 5.0;
        // A 1.2 km survey: the memory budget alone would allow one tile, or a few,
        // since a tile may be up to 512 cells (2.5 km) across here.
        big.bounds = Bounds3D(Vec3(-600.0, -600.0, -30.0), Vec3(600.0, 600.0, 0.0));

        Options base;
        apply_spacing_driven_defaults(base, &big);

        std::size_t previous_tiles = 0;
        for (const std::size_t threads : {1, 4, 8}) {
            Options options = base;
            options.threads = threads;
            options.max_chunks = 1000000;   // forces chunking on without fixing the tile size
            std::string chunk_error;
            if (!check(resolve_chunking(options, big, nullptr, &chunk_error) && options.use_chunking, "Chunking should be on with an automatically chosen tile size")) {
                return 1;
            }
            if (!check(options.chunking.threads == threads, "Chunking should use the requested thread count")) {
                return 1;
            }

            const double tile = options.chunking.tile_size;
            const double tiles_x = std::floor(big.bounds.max.x / tile) - std::floor(big.bounds.min.x / tile) + 1.0;
            const double tiles_y = std::floor(big.bounds.max.y / tile) - std::floor(big.bounds.min.y / tile) + 1.0;
            const std::size_t tiles = static_cast<std::size_t>(tiles_x * tiles_y);
            std::printf("  %zu threads: tile %.0f m, %zu tiles\n", threads, tile, tiles);

            if (!check(tiles >= 2 * threads, "Automatic tiling should give at least two tiles per thread")) {
                return 1;
            }
            if (!check(tiles >= previous_tiles, "Asking for more threads should not give fewer tiles")) {
                return 1;
            }
            previous_tiles = tiles;
        }

        // An explicit --chunk-size is honoured, not adjusted for threads.
        Options explicit_options = base;
        explicit_options.threads = 8;
        explicit_options.chunk_size_requested = true;
        explicit_options.chunk_size = 2500.0;
        std::string explicit_error;
        if (!check(resolve_chunking(explicit_options, big, nullptr, &explicit_error) &&
                       explicit_options.chunking.tile_size >= 2500.0 &&
                       explicit_options.chunking.tile_size < 2500.0 + base.poisson.cell_size,
                   "An explicit --chunk-size should be used as given (rounded up to whole cells)")) {
            return 1;
        }
    }

    std::printf("Settings defaults and Poisson budget checks passed\n");
    return 0;
}
