#include "algorithms/reconstruction.h"

#include "util/logger.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

namespace {

// A tile grid larger than this is refused outright rather than risking the
// machine; callers are expected to choose a tile size well below it.
constexpr double maximum_tile_grid_cells = 1.5e8;

// Prefixes a message with the tile it belongs to, so lines from tiles being
// reconstructed at the same time can be told apart.
[[nodiscard]] std::string labeled(const std::string &label, const std::string &message) {
    return label.empty() ? message : label + ": " + message;
}

[[nodiscard]] double snap_down(double value, double cell_size) {
    return std::floor(value / cell_size) * cell_size;
}

[[nodiscard]] double snap_up(double value, double cell_size) {
    return std::ceil(value / cell_size) * cell_size;
}

} // namespace

const char *reconstruction_status_message(ReconstructionStatus status) {
    switch (status) {
    case ReconstructionStatus::Ok:
        return "ok";
    case ReconstructionStatus::NoOrientedSamples:
        return "normal estimation produced no oriented samples";
    case ReconstructionStatus::EmptyField:
        return "screened Poisson reconstruction produced an empty field";
    case ReconstructionStatus::EmptyMesh:
        return "marching cubes produced an empty mesh";
    case ReconstructionStatus::TrimmedAway:
        return "support trimming removed the entire mesh; adjust the trimming thresholds or disable trimming";
    }
    return "unknown reconstruction failure";
}

ReconstructionResult reconstruct_chunk(
    const CollectedPointCloud &points,
    const ReconstructionOptions &options,
    const std::string &label) {

    ReconstructionResult result;

    // ------------------------------------------------------------------ normal estimation
    {
        const bool using_search_radius = options.normals.search_radius > 0.0;
        std::ostringstream message;
        message << "starting: search_radius=" << options.normals.search_radius
                << ", radius_mode=" << (using_search_radius ? "enabled" : "disabled")
                << ", k_nearest=" << options.normals.k
                << ", fallback_min_neighbors=" << options.normals.minimum_neighbors
                << ", fallback_to_knearest="
                << (using_search_radius ? "when radius neighborhood < minimum_neighbors" : "always");
        LOG_DEBUG("[normal-estimation]", labeled(label, message.str()));
    }

    result.oriented_points = normal_estimation(points, options.normals);
    if (result.oriented_points.empty()) {
        result.status = ReconstructionStatus::NoOrientedSamples;
        return result;
    }
    LOG_DEBUG("[normal-estimation]", labeled(label, "completed: oriented " + std::to_string(result.oriented_points.size()) + " samples"));

    // ------------------------------------------------------------------ screened Poisson
    {
        std::ostringstream message;
        message << "starting: cell_size=" << options.poisson.cell_size
                << ", padding=" << options.poisson.padding
                << ", splat_radius=" << options.poisson.normal_splat_radius
                << ", iterations=" << options.poisson.solver_iterations
                << ", screening=" << (options.poisson.use_screening ? "enabled" : "disabled")
                << ", screening_weight=" << options.poisson.screening_weight;
        LOG_DEBUG("[poisson]", labeled(label, message.str()));
    }

    ScalarGrid3D poisson_surface = screened_poisson(result.oriented_points, options.poisson);
    if (poisson_surface.values.empty()) {
        result.status = ReconstructionStatus::EmptyField;
        return result;
    }
    LOG_DEBUG("[poisson]", labeled(label, "completed: grid=" + std::to_string(poisson_surface.nx) + "x" + std::to_string(poisson_surface.ny) + "x" +
               std::to_string(poisson_surface.nz)));

    // ------------------------------------------------------------------ marching cubes
    LOG_DEBUG("[marching-cubes]", labeled(label, "starting: iso_value=" + std::to_string(options.marching_cubes.iso_value)));

    result.raw_mesh = marching_cubes(poisson_surface, options.marching_cubes);

    // The scalar grid is by far the largest object in the pipeline; release it
    // before trimming rather than at the end of the function.
    poisson_surface = ScalarGrid3D();

    if (result.raw_mesh.vertices.empty() || result.raw_mesh.indices.empty()) {
        result.status = ReconstructionStatus::EmptyMesh;
        return result;
    }
    LOG_DEBUG("[marching-cubes]", labeled(label, "completed: " + std::to_string(result.raw_mesh.vertices.size()) + " vertices, " +
               std::to_string(result.raw_mesh.indices.size() / 3) + " triangles"));

    // ------------------------------------------------------------------ support trimming
    if (options.trimming.enabled) {
        std::ostringstream message;
        message << "starting: radius="
                << ((options.trimming.support_radius > 0.0) ? std::to_string(options.trimming.support_radius) : "auto")
                << ", normal_offset="
                << ((options.trimming.max_normal_offset > 0.0) ? std::to_string(options.trimming.max_normal_offset) : "auto")
                << ", minimum_neighbors=" << options.trimming.minimum_neighbors
                << ", minimum_normal_alignment="
                << ((options.trimming.minimum_normal_alignment > 0.0) ? std::to_string(options.trimming.minimum_normal_alignment) : "disabled");
        LOG_DEBUG("[support-trimming]", labeled(label, message.str()));

        result.mesh = support_trimming(result.raw_mesh, result.oriented_points, options.trimming, &result.trimming_diagnostics);
    } else {
        LOG_DEBUG("[support-trimming]", labeled(label, "disabled; using raw mesh"));
        result.mesh = result.raw_mesh;
    }

    if (result.mesh.vertices.empty() || result.mesh.indices.empty()) {
        result.status = ReconstructionStatus::TrimmedAway;
        return result;
    }

    if (options.trimming.enabled) {
        const SupportTrimmingDiagnostics &diagnostics = result.trimming_diagnostics;
        std::ostringstream message;
        message << "completed: spacing=" << diagnostics.estimated_point_spacing
                << ", resolved_radius=" << diagnostics.resolved_support_radius
                << ", resolved_normal_offset=" << diagnostics.resolved_max_normal_offset
                << ", supported_vertices=" << diagnostics.supported_vertices << '/' << diagnostics.input_vertices
                << ", rejected=(neighbors=" << diagnostics.rejected_for_neighbors
                << ", offset=" << diagnostics.rejected_for_normal_offset
                << ", alignment=" << diagnostics.rejected_for_normal_alignment << ")"
                << ", output=" << result.mesh.vertices.size() << " vertices, "
                << result.mesh.indices.size() / 3 << " triangles";
        LOG_DEBUG("[support-trimming]", labeled(label, message.str()));
    }

    result.status = ReconstructionStatus::Ok;
    return result;
}

/******************************************************************************/

double round_to_cells(double length, double cell_size) {
    if (cell_size <= 0.0) {
        return length;
    }
    return std::max(1.0, std::ceil(length / cell_size - 1.0e-9)) * cell_size;
}

double chunk_halo(const ReconstructionOptions &options) {
    const double cell = options.poisson.cell_size;

    // Points within `reach` of a tile can affect its triangles, through normal
    // splatting or through support trimming.
    const double trimming_reach =
        options.trimming.enabled ? (options.trimming.support_radius > 0.0 ? options.trimming.support_radius : 6.0 * cell) : 0.0;
    const double reach = std::max(options.poisson.normal_splat_radius, trimming_reach);

    // Those points need correct normals, which need their own neighbors.
    const double normal_reach = options.normals.search_radius > 0.0 ? options.normals.search_radius : 4.0 * cell;

    return round_to_cells(reach + normal_reach + 4.0 * cell, cell);
}

Mesh crop_mesh_to_tile(const Mesh &mesh, double min_x, double max_x, double min_y, double max_y) {
    Mesh cropped;
    const bool has_normals = mesh.normals.size() == mesh.vertices.size();
    constexpr unsigned int unmapped = std::numeric_limits<unsigned int>::max();
    std::vector<unsigned int> remap(mesh.vertices.size(), unmapped);

    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const unsigned int a = mesh.indices[t];
        const unsigned int b = mesh.indices[t + 1];
        const unsigned int c = mesh.indices[t + 2];

        const double centroid_x = (mesh.vertices[a].x + mesh.vertices[b].x + mesh.vertices[c].x) / 3.0;
        const double centroid_y = (mesh.vertices[a].y + mesh.vertices[b].y + mesh.vertices[c].y) / 3.0;
        if (centroid_x < min_x || centroid_x >= max_x || centroid_y < min_y || centroid_y >= max_y) {
            continue;
        }

        for (const unsigned int old_index : {a, b, c}) {
            if (remap[old_index] == unmapped) {
                remap[old_index] = static_cast<unsigned int>(cropped.vertices.size());
                cropped.vertices.push_back(mesh.vertices[old_index]);
                if (has_normals) {
                    cropped.normals.push_back(mesh.normals[old_index]);
                }
            }
            cropped.indices.push_back(remap[old_index]);
        }
    }
    return cropped;
}

bool reconstruct_chunked(
    const CollectedPointCloud &points,
    const ReconstructionOptions &options,
    const ChunkingOptions &chunking,
    const TileCallback &on_tile,
    ChunkingStats *stats,
    std::string *error) {

    ChunkingStats local_stats;
    ChunkingStats &counts = stats != nullptr ? *stats : local_stats;
    counts = ChunkingStats();

    const double cell = options.poisson.cell_size;
    const double tile = chunking.tile_size;
    const double halo = chunking.halo;

    if (cell <= 0.0 || tile <= 0.0 || halo < 0.0) {
        if (error != nullptr) {
            *error = "chunked reconstruction needs a positive cell size and tile size and a non-negative halo";
        }
        return false;
    }
    if (points.size() >= std::numeric_limits<std::uint32_t>::max()) {
        if (error != nullptr) {
            *error = "chunked reconstruction supports fewer than 2^32 points";
        }
        return false;
    }

    // ---------------------------------------------------------------- bin points into tiles
    //
    // A point is stored in the tile that owns it and in every neighboring tile
    // whose halo reaches it. Only indices are stored, not copies of points.

    struct Tile {
        std::vector<std::uint32_t> indices;
        std::size_t owned = 0;
    };
    std::map<std::pair<int, int>, Tile> tiles;

    for (std::size_t n = 0; n < points.size(); n++) {
        const double x = points[n].point.x;
        const double y = points[n].point.y;

        const int i_first = static_cast<int>(std::floor((x - halo) / tile)) - 1;
        const int i_last = static_cast<int>(std::floor((x + halo) / tile)) + 1;
        const int j_first = static_cast<int>(std::floor((y - halo) / tile)) - 1;
        const int j_last = static_cast<int>(std::floor((y + halo) / tile)) + 1;

        for (int i = i_first; i <= i_last; i++) {
            if (x < i * tile - halo || x >= (i + 1) * tile + halo) {
                continue;
            }
            for (int j = j_first; j <= j_last; j++) {
                if (y < j * tile - halo || y >= (j + 1) * tile + halo) {
                    continue;
                }
                if (chunking.single_tile && (i != chunking.tile_i || j != chunking.tile_j)) {
                    continue;
                }

                Tile &entry = tiles[{i, j}];
                entry.indices.push_back(static_cast<std::uint32_t>(n));
                if (x >= i * tile && x < (i + 1) * tile && y >= j * tile && y < (j + 1) * tile) {
                    entry.owned++;
                }
            }
        }
    }

    for (const auto &entry : tiles) {
        if (entry.second.owned > 0) {
            counts.tiles_with_points++;
        }
    }
    LOG_DEBUG("[chunking]", "tile_size=" + std::to_string(tile) + " m, halo=" + std::to_string(halo) + " m, " +
               std::to_string(counts.tiles_with_points) + " tiles contain points");

    // ---------------------------------------------------------------- choose the tiles to run
    struct Job {
        int i = 0;
        int j = 0;
        std::size_t owned = 0;
        std::vector<std::uint32_t> indices;
    };
    std::vector<Job> jobs;

    for (auto &entry : tiles) {
        Tile &current = entry.second;
        if (current.owned == 0) {
            continue;   // the ring of halo-only tiles around the survey
        }
        if (current.owned < chunking.minimum_tile_points) {
            counts.tiles_skipped_sparse++;
            LOG_DEBUG("[chunking]", "tile " + std::to_string(entry.first.first) + "," + std::to_string(entry.first.second) +
                                        " skipped: only " + std::to_string(current.owned) + " points");
            continue;
        }
        if (chunking.max_tiles > 0 && jobs.size() >= chunking.max_tiles) {
            break;
        }
        jobs.push_back({entry.first.first, entry.first.second, current.owned, std::move(current.indices)});
    }
    tiles.clear();

    // ---------------------------------------------------------------- reconstruct tiles, possibly in parallel
    //
    // Workers take tiles in order. Finished tiles are handed to on_tile strictly
    // in tile order, so the output does not depend on the thread count.

    enum class Disposition { Pending, Mesh, Failed, EmptyCrop };
    struct Outcome {
        Disposition disposition = Disposition::Pending;
        TileResult tile;
    };
    std::vector<Outcome> outcomes(jobs.size());

    std::mutex state_mutex;
    std::size_t next_job = 0;
    std::size_t next_emit = 0;
    bool stop = false;
    bool failed = false;
    std::string failure;

    auto run_job = [&](Job &job, Outcome &outcome) -> bool {
        const int i = job.i;
        const int j = job.j;
        const std::string name = "tile " + std::to_string(i) + "," + std::to_string(j);

        CollectedPointCloud chunk_points;
        chunk_points.reserve(job.indices.size());
        double min_z = std::numeric_limits<double>::max();
        double max_z = std::numeric_limits<double>::lowest();
        for (const std::uint32_t index : job.indices) {
            chunk_points.push_back(points[index]);
            min_z = std::min(min_z, points[index].point.z);
            max_z = std::max(max_z, points[index].point.z);
        }
        std::vector<std::uint32_t>().swap(job.indices);

        // The grid covers the tile, its halo and the Poisson padding, with every
        // face on the global cell lattice so neighboring tiles share grid nodes.
        const double xy_margin = halo + options.poisson.padding;
        ReconstructionOptions tile_options = options;
        tile_options.poisson.use_fixed_domain = true;
        tile_options.poisson.domain_bounds = Bounds3D(
            Vec3(snap_down(i * tile - xy_margin, cell),
                 snap_down(j * tile - xy_margin, cell),
                 snap_down(min_z - options.poisson.padding, cell)),
            Vec3(snap_up((i + 1) * tile + xy_margin, cell),
                 snap_up((j + 1) * tile + xy_margin, cell),
                 snap_up(max_z + options.poisson.padding, cell)));

        const Vec3 size = tile_options.poisson.domain_bounds.size();
        const double grid_cells = (size.x / cell + 1.0) * (size.y / cell + 1.0) * (size.z / cell + 1.0);
        if (grid_cells > maximum_tile_grid_cells) {
            std::ostringstream message;
            message << name << " needs a Poisson grid of " << grid_cells << " cells, which exceeds "
                    << maximum_tile_grid_cells << "; use a smaller --chunk-size or a coarser level of detail";
            std::lock_guard<std::mutex> lock(state_mutex);
            failed = true;
            failure = message.str();
            return false;
        }

        LOG_DEBUG("[chunking]", name + ": " + std::to_string(job.owned) + " owned of " + std::to_string(chunk_points.size()) +
                       " points, grid " + std::to_string(static_cast<long long>(grid_cells)) + " cells");

        TileResult &tile_result = outcome.tile;
        tile_result.i = i;
        tile_result.j = j;
        tile_result.owned_points = job.owned;
        tile_result.input_points = chunk_points.size();
        tile_result.result = reconstruct_chunk(chunk_points, tile_options, name);
        chunk_points = CollectedPointCloud();

        if (tile_result.result.status != ReconstructionStatus::Ok) {
            LOG_DEBUG("[chunking]", name + " produced no mesh: " + reconstruction_status_message(tile_result.result.status));
            outcome.disposition = Disposition::Failed;
            tile_result.result = ReconstructionResult();
            return true;
        }

        // Keep only what this tile owns.
        const double min_x = i * tile;
        const double max_x = (i + 1) * tile;
        const double min_y = j * tile;
        const double max_y = (j + 1) * tile;

        ReconstructionResult &result = tile_result.result;
        result.mesh = crop_mesh_to_tile(result.mesh, min_x, max_x, min_y, max_y);
        result.raw_mesh = crop_mesh_to_tile(result.raw_mesh, min_x, max_x, min_y, max_y);

        OrientedPointCloud owned_oriented;
        for (const OrientedPoint &oriented : result.oriented_points) {
            if (oriented.point.x >= min_x && oriented.point.x < max_x && oriented.point.y >= min_y && oriented.point.y < max_y) {
                owned_oriented.push_back(oriented);
            }
        }
        result.oriented_points = std::move(owned_oriented);

        if (result.mesh.indices.empty()) {
            LOG_DEBUG("[chunking]", name + " has no triangles inside the tile");
            outcome.disposition = Disposition::EmptyCrop;
            tile_result.result = ReconstructionResult();
            return true;
        }

        LOG_DEBUG("[chunking]", name + " completed: " + std::to_string(result.mesh.vertices.size()) + " vertices, " +
                       std::to_string(result.mesh.indices.size() / 3) + " triangles");
        outcome.disposition = Disposition::Mesh;
        return true;
    };

    auto worker = [&]() {
        while (true) {
            std::size_t n = 0;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                if (stop || failed || next_job >= jobs.size()) {
                    return;
                }
                n = next_job++;
            }

            Outcome outcome;
            if (!run_job(jobs[n], outcome)) {
                return;
            }

            std::lock_guard<std::mutex> lock(state_mutex);
            outcomes[n] = std::move(outcome);
            while (!stop && next_emit < outcomes.size() && outcomes[next_emit].disposition != Disposition::Pending) {
                Outcome &ready = outcomes[next_emit++];
                switch (ready.disposition) {
                case Disposition::Failed:
                    counts.tiles_failed++;
                    break;
                case Disposition::EmptyCrop:
                    counts.tiles_empty_after_crop++;
                    break;
                case Disposition::Mesh:
                    counts.tiles_processed++;
                    if (on_tile && !on_tile(ready.tile)) {
                        stop = true;
                    }
                    break;
                case Disposition::Pending:
                    break;
                }
                ready.tile = TileResult();   // release the tile's meshes
            }
        }
    };

    const std::size_t thread_count = std::max<std::size_t>(1, std::min(chunking.threads, jobs.size()));
    if (thread_count == 1) {
        worker();
    }
    else {
        std::vector<std::thread> pool;
        pool.reserve(thread_count);
        for (std::size_t t = 0; t < thread_count; t++) {
            pool.emplace_back(worker);
        }
        for (std::thread &thread : pool) {
            thread.join();
        }
    }

    if (failed) {
        if (error != nullptr) {
            *error = failure;
        }
        return false;
    }
    return true;
}
