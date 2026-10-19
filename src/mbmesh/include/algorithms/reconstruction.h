#pragma once

#include <cstddef>
#include <functional>
#include <string>

#include "../data_types/geometry.h"
#include "marching_cubes.h"
#include "normal_estimation.h"
#include "screened_poisson.h"
#include "support_trimming.h"

/*******************************************************************************
 * Reconstruction of a surface mesh from a set of collected points, and a
 * chunked driver that reconstructs a large survey tile by tile.
 *
 * Chunking bounds memory and run time. The screened-Poisson grid covers the
 * bounding box of its input, so a single run over a whole survey needs a grid
 * that is mostly empty, and its cost grows quickly with size. Here the survey
 * is split into square horizontal tiles on a fixed lattice anchored at the
 * local-frame origin. Each tile is reconstructed from its own points plus a
 * halo of neighboring points, and keeps only the triangles it owns.
 ******************************************************************************/

struct ReconstructionOptions {
    NormalEstimationOptions normals;
    ScreenedPoissonOptions poisson;
    MarchingCubesOptions marching_cubes;
    SupportTrimmingOptions trimming;
};

enum class ReconstructionStatus {
    Ok,
    NoOrientedSamples,
    EmptyField,
    EmptyMesh,
    TrimmedAway,
};

[[nodiscard]] const char *reconstruction_status_message(ReconstructionStatus status);

struct ReconstructionResult {
    ReconstructionStatus status = ReconstructionStatus::Ok;
    Mesh raw_mesh;
    Mesh mesh;
    OrientedPointCloud oriented_points;
    SupportTrimmingDiagnostics trimming_diagnostics;
};

// Normal estimation, screened Poisson, marching cubes and support trimming for
// one set of points. Decimation is not included; apply it beforehand. Progress
// is logged at DEBUG level through util/logger.h; label, if given, prefixes each
// message (the chunked driver passes the tile name).
[[nodiscard]] ReconstructionResult reconstruct_chunk(
    const CollectedPointCloud &points,
    const ReconstructionOptions &options,
    const std::string &label = "");

/******************************************************************************/

struct ChunkingOptions {
    // Edge length of a tile, in meters. Must be a whole number of Poisson
    // cells; use round_to_cells() to make it so.
    double tile_size = 0.0;

    // Width of the ring of neighboring points each tile also reads, in meters.
    // Use chunk_halo() to compute a value that covers every stage's reach.
    double halo = 0.0;

    // Tiles with fewer owned points than this are skipped.
    std::size_t minimum_tile_points = 32;

    // Number of tiles reconstructed at the same time. Memory use grows with
    // this, since each tile holds its own Poisson grid. Tiles are delivered to
    // the callback in tile order whatever the thread count.
    std::size_t threads = 1;

    // Testing aids: process only one tile, or stop after a number of tiles.
    bool single_tile = false;
    int tile_i = 0;
    int tile_j = 0;
    std::size_t max_tiles = 0;   // 0 = no limit
};

struct ChunkingStats {
    std::size_t tiles_with_points = 0;
    std::size_t tiles_processed = 0;
    std::size_t tiles_skipped_sparse = 0;
    std::size_t tiles_failed = 0;
    std::size_t tiles_empty_after_crop = 0;
};

// One reconstructed tile. The meshes are cropped to the tile (see below) and
// oriented_points holds only the points the tile owns.
struct TileResult {
    int i = 0;
    int j = 0;
    std::size_t owned_points = 0;
    std::size_t input_points = 0;
    ReconstructionResult result;
};

// Return false to stop processing.
using TileCallback = std::function<bool(TileResult &)>;

// Smallest halo, in meters, that lets a tile's owned region see the same
// neighborhoods a single run would: the reach of splatting or trimming, plus
// the normal-estimation radius needed to orient points at that reach, plus a
// few cells of margin.
[[nodiscard]] double chunk_halo(const ReconstructionOptions &options);

// Rounds a length up to a whole number of Poisson cells (at least one).
[[nodiscard]] double round_to_cells(double length, double cell_size);

// Tile (i, j) owns x in [i*size, (i+1)*size) and y in [j*size, (j+1)*size) in
// the local frame. A triangle belongs to the tile that contains its centroid,
// so every triangle belongs to exactly one tile.
[[nodiscard]] Mesh crop_mesh_to_tile(const Mesh &mesh, double min_x, double max_x, double min_y, double max_y);

// Reconstructs a survey tile by tile, calling on_tile for each tile that
// produces a mesh, in order of tile index. With more than one thread the
// callback is still called one tile at a time. Returns false only on a hard
// error.
[[nodiscard]] bool reconstruct_chunked(
    const CollectedPointCloud &points,
    const ReconstructionOptions &options,
    const ChunkingOptions &chunking,
    const TileCallback &on_tile,
    ChunkingStats *stats,
    std::string *error);
