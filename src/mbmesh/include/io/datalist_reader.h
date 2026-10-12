#pragma once

#include <cstddef>
#include <filesystem>
#include <string>

#include "../data_types/geometry.h"
#include "../data_types/swathfile.h"
#include "coordinate_frame.h"

struct DatalistReaderOptions {
    GeographicBounds bounds;
    bool use_bounds = false;
    bool reject_flagged_beams = true;
    int verbose = 0;
};

struct DatalistReaderStats {
    std::size_t files_seen = 0;
    std::size_t files_read = 0;
    std::size_t pings_read = 0;
    std::size_t soundings_read = 0;
    std::size_t soundings_accepted = 0;
    std::size_t soundings_flagged = 0;
    std::size_t soundings_out_of_bounds = 0;
};

struct DatalistReadResult {
    CollectedPointCloud points;
    CoordinateFrame frame;
    DatalistReaderStats stats;
};

[[nodiscard]] bool read_datalist(
    const std::filesystem::path &path,
    const DatalistReaderOptions &options,
    DatalistReadResult *result,
    std::string *error);
