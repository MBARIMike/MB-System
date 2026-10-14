#include "algorithms/reconstruction.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double survey_size = 30.0;     // meters, square
constexpr double tile_size = 10.0;       // meters, 3 x 3 tiles
constexpr double sample_spacing = 0.25;

double surface_height(double x, double y) {
    return 1.5 * std::sin(0.35 * x) * std::cos(0.25 * y) + 0.2 * x * 0.05;
}

bool check(bool condition, const std::string &message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message.c_str());
    }
    return condition;
}

// A survey of a gently varying seafloor with the sonar 20 m above it, so every
// normal is oriented upward.
CollectedPointCloud make_survey() {
    CollectedPointCloud points;
    const int count = static_cast<int>(std::round(survey_size / sample_spacing));
    for (int ix = 0; ix < count; ix++) {
        for (int iy = 0; iy < count; iy++) {
            const double x = ix * sample_spacing;
            const double y = iy * sample_spacing;
            const double z = surface_height(x, y);
            points.push_back({Vec3(x, y, z), Vec3(x, y, z + 20.0)});
        }
    }
    return points;
}

ReconstructionOptions make_options() {
    ReconstructionOptions options;
    options.normals.k = 24;
    options.normals.search_radius = 0.8;
    options.normals.minimum_neighbors = 6;

    options.poisson.cell_size = 0.5;
    options.poisson.padding = 2.0;
    options.poisson.normal_splat_radius = 1.0;
    options.poisson.screening_weight = 3.0;
    options.poisson.solver_iterations = 300;
    options.poisson.use_screening = true;
    options.poisson.iso_value = 0.0;

    options.marching_cubes.iso_value = 0.0;

    options.trimming.enabled = true;
    options.trimming.support_radius = 1.5;
    options.trimming.max_normal_offset = 0.5;
    options.trimming.minimum_neighbors = 3;
    return options;
}

double max_height_error(const Mesh &mesh) {
    double worst = 0.0;
    for (const Vec3 &vertex : mesh.vertices) {
        worst = std::max(worst, std::fabs(vertex.z - surface_height(vertex.x, vertex.y)));
    }
    return worst;
}

// True if (x, y) lies inside some triangle of the mesh, seen from above.
bool covered(const Mesh &mesh, double x, double y) {
    constexpr double slack = 1.0e-9;
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const Vec3 &a = mesh.vertices[mesh.indices[t]];
        const Vec3 &b = mesh.vertices[mesh.indices[t + 1]];
        const Vec3 &c = mesh.vertices[mesh.indices[t + 2]];
        const double d1 = (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x);
        const double d2 = (c.x - b.x) * (y - b.y) - (c.y - b.y) * (x - b.x);
        const double d3 = (a.x - c.x) * (y - c.y) - (a.y - c.y) * (x - c.x);
        const bool has_negative = d1 < -slack || d2 < -slack || d3 < -slack;
        const bool has_positive = d1 > slack || d2 > slack || d3 > slack;
        if (!(has_negative && has_positive)) {
            return true;
        }
    }
    return false;
}

void append(Mesh *destination, const Mesh &source) {
    const unsigned int offset = static_cast<unsigned int>(destination->vertices.size());
    destination->vertices.insert(destination->vertices.end(), source.vertices.begin(), source.vertices.end());
    for (const unsigned int index : source.indices) {
        destination->indices.push_back(index + offset);
    }
}

} // namespace

int main() {
    bool ok = true;

    const CollectedPointCloud points = make_survey();
    const ReconstructionOptions options = make_options();

    // ---- helpers
    ok &= check(std::fabs(round_to_cells(10.1, 0.5) - 10.5) < 1e-12, "round_to_cells rounds up to whole cells");
    ok &= check(std::fabs(round_to_cells(0.01, 0.5) - 0.5) < 1e-12, "round_to_cells returns at least one cell");

    const double halo = chunk_halo(options);
    ok &= check(halo >= options.poisson.normal_splat_radius + options.normals.search_radius, "halo covers splat reach plus normal radius");
    ok &= check(std::fabs(std::remainder(halo, options.poisson.cell_size)) < 1e-9, "halo is a whole number of cells");

    {
        Mesh square;
        square.vertices = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(1, 1, 0), Vec3(0, 1, 0), Vec3(5, 5, 0), Vec3(6, 5, 0), Vec3(6, 6, 0)};
        square.indices = {0, 1, 2, 0, 2, 3, 4, 5, 6};
        const Mesh cropped = crop_mesh_to_tile(square, 0.0, 2.0, 0.0, 2.0);
        ok &= check(cropped.indices.size() == 6 && cropped.vertices.size() == 4, "crop keeps only triangles whose centroid is in the tile");
        const Mesh edge = crop_mesh_to_tile(square, 1.0, 2.0, 0.0, 2.0);
        ok &= check(edge.indices.empty() || edge.indices.size() == 3, "crop uses a half-open tile");
    }

    // ---- one run over the whole survey
    const ReconstructionResult whole = reconstruct_chunk(points, options);
    ok &= check(whole.status == ReconstructionStatus::Ok, "whole-survey reconstruction succeeds");
    const double whole_error = max_height_error(whole.mesh);
    std::printf("whole survey: %zu vertices, %zu triangles, max height error %.3f m\n",
                whole.mesh.vertices.size(), whole.mesh.indices.size() / 3, whole_error);

    // ---- the same survey in tiles
    ChunkingOptions chunking;
    chunking.tile_size = tile_size;
    chunking.halo = halo;

    Mesh merged;
    std::vector<TileResult> tile_meshes;
    ChunkingStats stats;
    std::string error;
    const bool chunked_ok = reconstruct_chunked(
        points, options, chunking, {},
        [&](TileResult &tile) {
            append(&merged, tile.result.mesh);
            tile_meshes.push_back(tile);
            return true;
        },
        &stats, &error);
    ok &= check(chunked_ok, "chunked reconstruction succeeds: " + error);
    ok &= check(stats.tiles_processed == 9, "a 30 m survey in 10 m tiles gives 3 x 3 tiles, got " + std::to_string(stats.tiles_processed));
    ok &= check(stats.tiles_failed == 0 && stats.tiles_skipped_sparse == 0, "no tile fails or is skipped");

    const double chunked_error = max_height_error(merged);
    std::printf("chunked:      %zu vertices, %zu triangles, max height error %.3f m (%zu tiles)\n",
                merged.vertices.size(), merged.indices.size() / 3, chunked_error, stats.tiles_processed);

    // Tiling must not make the surface noticeably worse than one run does.
    ok &= check(chunked_error <= whole_error + 0.5 * options.poisson.cell_size,
                "chunked surface is as accurate as the whole-survey surface (" + std::to_string(chunked_error) +
                    " vs " + std::to_string(whole_error) + ")");

    // Each tile owns only triangles whose centroids lie inside it.
    for (const TileResult &tile : tile_meshes) {
        const Mesh &mesh = tile.result.mesh;
        bool inside = true;
        for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            const Vec3 &a = mesh.vertices[mesh.indices[t]];
            const Vec3 &b = mesh.vertices[mesh.indices[t + 1]];
            const Vec3 &c = mesh.vertices[mesh.indices[t + 2]];
            const double cx = (a.x + b.x + c.x) / 3.0;
            const double cy = (a.y + b.y + c.y) / 3.0;
            inside = inside && cx >= tile.i * tile_size && cx < (tile.i + 1) * tile_size &&
                     cy >= tile.j * tile_size && cy < (tile.j + 1) * tile_size;
        }
        ok &= check(inside, "tile " + std::to_string(tile.i) + "," + std::to_string(tile.j) + " owns only triangles inside it");
    }

    // No gaps: sample points away from the survey's outer edge, including
    // points right on the seams, must be covered by some triangle.
    int uncovered = 0;
    int samples = 0;
    for (double x = 4.0; x <= survey_size - 4.0; x += 0.7) {
        for (double y = 4.0; y <= survey_size - 4.0; y += 0.7) {
            samples++;
            if (!covered(merged, x, y)) {
                uncovered++;
            }
        }
    }
    for (const double seam : {10.0, 20.0}) {
        for (double along = 4.0; along <= survey_size - 4.0; along += 0.7) {
            samples += 2;
            if (!covered(merged, seam, along)) {
                uncovered++;
            }
            if (!covered(merged, along, seam)) {
                uncovered++;
            }
        }
    }
    std::printf("coverage:     %d of %d interior samples uncovered\n", uncovered, samples);
    ok &= check(uncovered == 0, "merged tiles leave no gaps in the interior");

    // The thread count must not change the result: same tiles, in the same
    // order, with identical meshes.
    {
        ChunkingOptions threaded = chunking;
        threaded.threads = 4;
        std::vector<std::pair<int, int>> order;
        Mesh threaded_merged;
        ok &= check(reconstruct_chunked(points, options, threaded, {}, [&](TileResult &tile) {
                        order.push_back({tile.i, tile.j});
                        append(&threaded_merged, tile.result.mesh);
                        return true;
                    }, &stats, &error),
                    "threaded chunked reconstruction succeeds: " + error);

        bool same_order = order.size() == tile_meshes.size();
        for (std::size_t n = 0; same_order && n < order.size(); n++) {
            same_order = order[n].first == tile_meshes[n].i && order[n].second == tile_meshes[n].j;
        }
        ok &= check(same_order, "tiles are delivered in tile order whatever the thread count");

        bool same_mesh = threaded_merged.vertices.size() == merged.vertices.size() &&
                         threaded_merged.indices == merged.indices;
        for (std::size_t n = 0; same_mesh && n < merged.vertices.size(); n++) {
            same_mesh = merged.vertices[n].x == threaded_merged.vertices[n].x &&
                        merged.vertices[n].y == threaded_merged.vertices[n].y &&
                        merged.vertices[n].z == threaded_merged.vertices[n].z;
        }
        ok &= check(same_mesh, "four threads produce exactly the same mesh as one");

        // Stopping from the callback stops the run.
        std::size_t seen = 0;
        ok &= check(reconstruct_chunked(points, options, threaded, {}, [&](TileResult &) {
                        seen++;
                        return false;
                    }, &stats, &error) && seen == 1,
                    "a callback returning false stops processing");
    }

    // Single-tile and tile-count limits.
    {
        ChunkingOptions one = chunking;
        one.single_tile = true;
        one.tile_i = 1;
        one.tile_j = 1;
        std::size_t seen = 0;
        ok &= check(reconstruct_chunked(points, options, one, {}, [&](TileResult &tile) {
                        seen++;
                        return tile.i == 1 && tile.j == 1;
                    }, &stats, &error) && seen == 1,
                    "--chunk selects exactly one tile");

        ChunkingOptions limited = chunking;
        limited.max_tiles = 2;
        seen = 0;
        ok &= check(reconstruct_chunked(points, options, limited, {}, [&](TileResult &) {
                        seen++;
                        return true;
                    }, &stats, &error) && seen == 2,
                    "--max-chunks limits the number of tiles");
    }

    std::printf(ok ? "chunking tests passed\n" : "chunking tests FAILED\n");
    return ok ? 0 : 1;
}
