#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "../data_types/geometry.h"

[[nodiscard]] bool write_mesh_glb_file(
    const std::filesystem::path &path,
    const Mesh &mesh,
    std::string *error);

[[nodiscard]] bool write_pointcloud_glb_file(
    const std::filesystem::path &path,
    const PointCloud &pointcloud,
    std::string *error);

[[nodiscard]] bool write_normal_lines_glb_file(
    const std::filesystem::path &path,
    const OrientedPointCloud &pointcloud,
    double line_length,
    std::string *error);

// Draws polylines, for example the sensor track, as line segments. Positions are
// written as given, so they must already be in the output frame.
[[nodiscard]] bool write_track_glb_file(
    const std::filesystem::path &path,
    const std::vector<std::vector<Vec3>> &polylines,
    std::string *error);

[[nodiscard]] bool write_origin_ray_lines_glb_file(
    const std::filesystem::path &path,
    const CollectedPointCloud &pointcloud,
    double line_length,
    std::string *error);
