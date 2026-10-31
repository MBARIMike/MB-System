#include "io/point_spill.h"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <sys/types.h>
#include <system_error>

namespace {

// Points held by the spill writer before they are written.
constexpr std::size_t spill_buffer_points = 1 << 20;

void encode_record(unsigned char *at, const Vec3 &point, std::uint32_t ping) {
    std::memcpy(at, &point.x, sizeof(double));
    std::memcpy(at + sizeof(double), &point.y, sizeof(double));
    std::memcpy(at + 2 * sizeof(double), &point.z, sizeof(double));
    std::memcpy(at + 3 * sizeof(double), &ping, sizeof(std::uint32_t));
}

// False if the ping is not in origins.
[[nodiscard]] bool decode_record(const unsigned char *at, const std::vector<Vec3> &origins, CollectedPoint *point) {
    std::uint32_t ping = 0;
    std::memcpy(&point->point.x, at, sizeof(double));
    std::memcpy(&point->point.y, at + sizeof(double), sizeof(double));
    std::memcpy(&point->point.z, at + 2 * sizeof(double), sizeof(double));
    std::memcpy(&ping, at + 3 * sizeof(double), sizeof(std::uint32_t));
    if (ping >= origins.size()) {
        return false;
    }
    point->origin = origins[ping];
    return true;
}

void set_error(std::string *error, const std::string &message) {
    if (error != nullptr) {
        *error = message;
    }
}

[[nodiscard]] std::string errno_text() {
    return std::strerror(errno);
}

// Closes a file when it goes out of scope.
struct FileCloser {
    std::FILE *file;
    explicit FileCloser(std::FILE *f) : file(f) {}
    ~FileCloser() {
        if (file != nullptr) {
            std::fclose(file);
        }
    }
};

} // namespace

PointSpill::~PointSpill() {
    if (file_ != nullptr) {
        std::fclose(file_);
    }
}

bool PointSpill::open(const std::filesystem::path &path, std::string *error) {
    if (file_ != nullptr) {
        std::fclose(file_);
        file_ = nullptr;
    }
    path_ = path;
    count_ = 0;
    buffer_.clear();
    buffer_.reserve(spill_buffer_points * spill_record_bytes);
    file_ = std::fopen(path.string().c_str(), "wb");
    if (file_ == nullptr) {
        set_error(error, "unable to create the file of spilled soundings '" + path.string() + "': " + errno_text());
        return false;
    }
    return true;
}

bool PointSpill::flush(std::string *error) {
    if (buffer_.empty()) {
        return true;
    }
    if (file_ == nullptr) {
        set_error(error, "the file of spilled soundings is not open");
        return false;
    }
    if (std::fwrite(buffer_.data(), 1, buffer_.size(), file_) != buffer_.size()) {
        set_error(error, "error writing the file of spilled soundings '" + path_.string() + "' after " + std::to_string(count_ - buffer_.size() / spill_record_bytes) +
                             " soundings (" + std::to_string((count_ - buffer_.size() / spill_record_bytes) * spill_record_bytes / 1000000000) + " GB): " + errno_text() +
                             "; the disk is probably full. The spill file needs " + std::to_string(spill_record_bytes) +
                             " bytes for each sounding, so use -O to put the results, and this file, on a disk with room");
        return false;
    }
    buffer_.clear();
    return true;
}

bool PointSpill::append(const Vec3 &point, std::uint32_t ping, std::string *error) {
    const std::size_t at = buffer_.size();
    buffer_.resize(at + spill_record_bytes);
    encode_record(buffer_.data() + at, point, ping);
    count_++;
    return buffer_.size() < spill_buffer_points * spill_record_bytes || flush(error);
}

bool PointSpill::finish(std::string *error) {
    bool ok = flush(error);
    if (file_ != nullptr) {
        if (std::fclose(file_) != 0 && ok) {
            set_error(error, "error closing the file of spilled soundings '" + path_.string() + "': " + errno_text());
            ok = false;
        }
        file_ = nullptr;
    }
    std::vector<unsigned char>().swap(buffer_);
    return ok;
}

bool read_spill_chunks(const std::filesystem::path &path, const std::vector<Vec3> &origins, std::size_t chunk_points, const SpillChunkCallback &callback,
                       std::string *error) {
    std::FILE *file = std::fopen(path.string().c_str(), "rb");
    if (file == nullptr) {
        set_error(error, "unable to open the file of spilled soundings '" + path.string() + "': " + errno_text());
        return false;
    }
    const FileCloser closer(file);

    chunk_points = std::max<std::size_t>(1, chunk_points);
    std::vector<unsigned char> bytes(chunk_points * spill_record_bytes);
    CollectedPointCloud chunk;
    std::size_t first = 0;
    while (true) {
        const std::size_t got = std::fread(bytes.data(), spill_record_bytes, chunk_points, file);
        if (got < chunk_points && std::ferror(file)) {
            set_error(error, "error reading the file of spilled soundings '" + path.string() + "': " + errno_text());
            return false;
        }
        if (got == 0) {
            return true;
        }
        chunk.resize(got);
        for (std::size_t n = 0; n < got; n++) {
            if (!decode_record(bytes.data() + n * spill_record_bytes, origins, &chunk[n])) {
                set_error(error, "the file of spilled soundings '" + path.string() + "' names a ping that the track does not have, at sounding " + std::to_string(first + n));
                return false;
            }
        }
        if (!callback(chunk, first)) {
            return true;
        }
        first += got;
        if (got < chunk_points) {
            return true;
        }
    }
}

bool read_spill_range(const std::filesystem::path &path, const std::vector<Vec3> &origins, std::size_t first, std::size_t count,
                      CollectedPointCloud *points, std::string *error) {
    points->clear();
    std::FILE *file = std::fopen(path.string().c_str(), "rb");
    if (file == nullptr) {
        set_error(error, "unable to open the file of spilled soundings '" + path.string() + "': " + errno_text());
        return false;
    }
    const FileCloser closer(file);

    if (fseeko(file, static_cast<off_t>(first) * static_cast<off_t>(spill_record_bytes), SEEK_SET) != 0) {
        set_error(error, "unable to seek in the file of spilled soundings '" + path.string() + "': " + errno_text());
        return false;
    }
    std::vector<unsigned char> bytes(count * spill_record_bytes);
    const std::size_t got = std::fread(bytes.data(), spill_record_bytes, count, file);
    if (got != count) {
        set_error(error, "the file of spilled soundings '" + path.string() + "' ends before point " + std::to_string(first + count));
        return false;
    }
    points->resize(count);
    for (std::size_t n = 0; n < count; n++) {
        if (!decode_record(bytes.data() + n * spill_record_bytes, origins, &(*points)[n])) {
            points->clear();
            set_error(error, "the file of spilled soundings '" + path.string() + "' names a ping that the track does not have, at sounding " + std::to_string(first + n));
            return false;
        }
    }
    return true;
}

TileBins::TileBins(const std::filesystem::path &directory, double tile_size, double halo, std::size_t memory_points)
    : directory_(directory), tile_size_(tile_size), halo_(halo), memory_points_(std::max<std::size_t>(1, memory_points)) {}

TileBins::~TileBins() = default;

std::filesystem::path TileBins::tile_path(int i, int j) const {
    return directory_ / ("tile_" + std::to_string(i) + "_" + std::to_string(j) + ".bin");
}

bool TileBins::open(std::string *error) {
    std::error_code filesystem_error;
    std::filesystem::remove_all(directory_, filesystem_error);
    filesystem_error.clear();
    std::filesystem::create_directories(directory_, filesystem_error);
    if (filesystem_error) {
        set_error(error, "unable to create the directory of tile files '" + directory_.string() + "': " + filesystem_error.message());
        return false;
    }
    counts_.clear();
    buffers_.clear();
    buffered_ = 0;
    return true;
}

bool TileBins::add(const CollectedPoint &point, std::string *error) {
    for_each_tile_of_point(point.point.x, point.point.y, tile_size_, halo_, [&](int i, int j, bool owned) {
        if (only_tile_ && (i != only_i_ || j != only_j_)) {
            return;
        }
        TileBinCount &count = counts_[{i, j}];
        count.total++;
        if (owned) {
            count.owned++;
        }
        buffers_[{i, j}].push_back(point);
        buffered_++;
    });
    return buffered_ < memory_points_ || flush(error);
}

bool TileBins::flush(std::string *error) {
    for (auto &entry : buffers_) {
        CollectedPointCloud &buffer = entry.second;
        if (buffer.empty()) {
            continue;
        }
        const std::filesystem::path path = tile_path(entry.first.first, entry.first.second);
        std::FILE *file = std::fopen(path.string().c_str(), "ab");
        if (file == nullptr) {
            set_error(error, "unable to open the tile file '" + path.string() + "': " + errno_text());
            return false;
        }
        const std::size_t written = std::fwrite(buffer.data(), spilled_point_bytes, buffer.size(), file);
        const bool closed = std::fclose(file) == 0;
        if (written != buffer.size() || !closed) {
            set_error(error, "error writing the tile file '" + path.string() + "': " + errno_text() + " (is the disk full?)");
            return false;
        }
        buffer.clear();
    }
    buffered_ = 0;
    return true;
}

bool TileBins::finish(std::string *error) {
    if (!flush(error)) {
        return false;
    }
    buffers_.clear();
    return true;
}

std::size_t TileBins::owned_point_count() const {
    std::size_t total = 0;
    for (const auto &entry : counts_) {
        total += entry.second.owned;
    }
    return total;
}

bool TileBins::read_tile(int i, int j, CollectedPointCloud *points, std::string *error) const {
    points->clear();
    const auto found = counts_.find({i, j});
    if (found == counts_.end() || found->second.total == 0) {
        return true;
    }
    const std::filesystem::path path = tile_path(i, j);
    std::FILE *file = std::fopen(path.string().c_str(), "rb");
    if (file == nullptr) {
        set_error(error, "unable to open the tile file '" + path.string() + "': " + errno_text());
        return false;
    }
    const FileCloser closer(file);
    points->resize(found->second.total);
    if (std::fread(points->data(), spilled_point_bytes, points->size(), file) != points->size()) {
        points->clear();
        set_error(error, "the tile file '" + path.string() + "' is shorter than the " + std::to_string(found->second.total) + " points it should hold");
        return false;
    }
    return true;
}

bool TileBins::for_each_owned(const std::function<bool(const CollectedPointCloud &)> &callback, std::string *error) const {
    CollectedPointCloud tile_points;
    CollectedPointCloud owned;
    for (const auto &entry : counts_) {
        if (entry.second.owned == 0) {
            continue;
        }
        const int i = entry.first.first;
        const int j = entry.first.second;
        if (!read_tile(i, j, &tile_points, error)) {
            return false;
        }
        owned.clear();
        for (const CollectedPoint &point : tile_points) {
            if (point.point.x >= i * tile_size_ && point.point.x < (i + 1) * tile_size_ && point.point.y >= j * tile_size_ &&
                point.point.y < (j + 1) * tile_size_) {
                owned.push_back(point);
            }
        }
        if (!callback(owned)) {
            return true;
        }
    }
    return true;
}

void TileBins::remove() {
    buffers_.clear();
    buffered_ = 0;
    std::error_code filesystem_error;
    std::filesystem::remove_all(directory_, filesystem_error);
}
