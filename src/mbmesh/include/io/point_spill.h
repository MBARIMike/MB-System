#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "../data_types/geometry.h"

/*******************************************************************************
 * Soundings on disk, for surveys that are too large to hold in memory.
 *
 * Two files are involved. The reader writes each accepted sounding to a spill
 * file as it is read (PointSpill), so that memory does not grow with the number
 * of soundings. Once the reconstruction frame, the point spacing and the tile
 * size are known, the points are read back in pieces and sorted into one file
 * per reconstruction tile (TileBins), each tile file holding the points of the
 * tile and of the halo around it, so that a tile is reconstructed from its own
 * file and only the tiles being worked on are in memory.
 *
 * The files are temporary and private to one run, in the host byte order. A tile
 * file record is the bytes of a CollectedPoint (two Vec3 of doubles, 48 bytes).
 * A spill file record is smaller, 28 bytes: the position of the sounding as
 * three doubles and the number of its ping, since the sensor position is the
 * same for all the soundings of a ping and is kept once, in the track.
 ******************************************************************************/

static_assert(std::is_trivially_copyable<CollectedPoint>::value, "CollectedPoint is written to disk as raw bytes");

// Bytes of one record in a tile file, and in the spill file.
constexpr std::size_t spilled_point_bytes = sizeof(CollectedPoint);
constexpr std::size_t spill_record_bytes = 3 * sizeof(double) + sizeof(std::uint32_t);

// Sequential writer of the spill file.
class PointSpill {
public:
    PointSpill() = default;
    PointSpill(const PointSpill &) = delete;
    PointSpill &operator=(const PointSpill &) = delete;
    ~PointSpill();

    // Creates (or empties) the file.
    [[nodiscard]] bool open(const std::filesystem::path &path, std::string *error);
    [[nodiscard]] bool is_open() const { return file_ != nullptr; }

    // ping is the index of the sounding's ping in the list of sensor positions that the points are read back with.
    [[nodiscard]] bool append(const Vec3 &point, std::uint32_t ping, std::string *error);

    // Writes what is buffered and closes the file.
    [[nodiscard]] bool finish(std::string *error);

    [[nodiscard]] std::size_t count() const { return count_; }
    [[nodiscard]] const std::filesystem::path &path() const { return path_; }

private:
    bool flush(std::string *error);

    std::FILE *file_ = nullptr;
    std::filesystem::path path_;
    std::vector<unsigned char> buffer_;   // whole records
    std::size_t count_ = 0;
};

// Reads a spill file in pieces of at most chunk_points points, calling the callback with each piece and the index in
// the file of its first point. The point of each is as it was written, and its origin is origins[ping]. The callback may
// change the piece. It returns false to stop, which is not an error.
using SpillChunkCallback = std::function<bool(CollectedPointCloud &chunk, std::size_t first_index)>;
[[nodiscard]] bool read_spill_chunks(const std::filesystem::path &path, const std::vector<Vec3> &origins, std::size_t chunk_points,
                                     const SpillChunkCallback &callback, std::string *error);

// Reads count points starting at index first (fewer, with an error, if the file is shorter).
[[nodiscard]] bool read_spill_range(const std::filesystem::path &path, const std::vector<Vec3> &origins, std::size_t first, std::size_t count,
                                    CollectedPointCloud *points, std::string *error);

// Calls visit(i, j, owned) for each tile of this size whose points include a point at x, y: the tile that owns it
// and every neighbor whose halo reaches it. Tile (i, j) owns x in [i*tile, (i+1)*tile) and y in [j*tile, (j+1)*tile).
// The in-memory chunker and the on-disk one both use this, so that they put every point in the same tiles.
template <typename Visit>
void for_each_tile_of_point(double x, double y, double tile, double halo, Visit &&visit) {
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
            const bool owned = x >= i * tile && x < (i + 1) * tile && y >= j * tile && y < (j + 1) * tile;
            visit(i, j, owned);
        }
    }
}

// How many points a tile file holds.
struct TileBinCount {
    std::size_t owned = 0;   // inside the tile
    std::size_t total = 0;   // with the halo
};

// One file of points for each reconstruction tile.
class TileBins {
public:
    // The files go in directory. If only_tile is set, only that tile is kept (--chunk, for testing).
    // memory_points bounds how many points are buffered in memory before they are appended to the files.
    TileBins(const std::filesystem::path &directory, double tile_size, double halo, std::size_t memory_points = 4000000);
    TileBins(const TileBins &) = delete;
    TileBins &operator=(const TileBins &) = delete;
    ~TileBins();

    void keep_only_tile(int i, int j) {
        only_tile_ = true;
        only_i_ = i;
        only_j_ = j;
    }

    // Creates the directory, and empties it of earlier tile files.
    [[nodiscard]] bool open(std::string *error);
    [[nodiscard]] bool add(const CollectedPoint &point, std::string *error);
    // Writes what is buffered. No more points can be added.
    [[nodiscard]] bool finish(std::string *error);

    [[nodiscard]] double tile_size() const { return tile_size_; }
    [[nodiscard]] double halo() const { return halo_; }
    [[nodiscard]] const std::map<std::pair<int, int>, TileBinCount> &tiles() const { return counts_; }
    // Points owned by tiles: each point of the survey once.
    [[nodiscard]] std::size_t owned_point_count() const;

    // The points of tile (i, j), its halo included, in the order they were added. Safe to call from several threads.
    [[nodiscard]] bool read_tile(int i, int j, CollectedPointCloud *points, std::string *error) const;

    // Calls the callback with the points each tile owns, one tile at a time, so that every point is visited once.
    // It returns false to stop, which is not an error.
    [[nodiscard]] bool for_each_owned(const std::function<bool(const CollectedPointCloud &)> &callback, std::string *error) const;

    // Deletes the tile files and the directory.
    void remove();

private:
    [[nodiscard]] std::filesystem::path tile_path(int i, int j) const;
    [[nodiscard]] bool flush(std::string *error);

    std::filesystem::path directory_;
    double tile_size_;
    double halo_;
    std::size_t memory_points_;
    bool only_tile_ = false;
    int only_i_ = 0;
    int only_j_ = 0;
    std::map<std::pair<int, int>, TileBinCount> counts_;
    std::map<std::pair<int, int>, CollectedPointCloud> buffers_;
    std::size_t buffered_ = 0;
};
