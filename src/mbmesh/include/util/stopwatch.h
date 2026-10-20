#pragma once

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

namespace mbmesh {

// Wall-clock timer for log messages.
class Stopwatch {
public:
    Stopwatch() : start_(std::chrono::steady_clock::now()) {}

    [[nodiscard]] double seconds() const {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    }

    // For example "12.3 s", or "2 min 5.1 s" once it passes a minute.
    [[nodiscard]] std::string text() const {
        const double total = seconds();
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(1);
        if (total >= 60.0) {
            const long minutes = static_cast<long>(total / 60.0);
            stream << minutes << " min " << (total - 60.0 * static_cast<double>(minutes)) << " s";
        } else {
            stream << total << " s";
        }
        return stream.str();
    }

private:
    std::chrono::steady_clock::time_point start_;
};

} // namespace mbmesh
