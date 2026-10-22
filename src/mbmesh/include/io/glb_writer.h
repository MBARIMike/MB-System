#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "../data_types/geometry.h"
#include "provenance.h"

// The georeferencing and provenance metadata of the run (see provenance.h and
// GEOREFERENCE.md). Once set, every GLB written gets it as asset.extras, with a
// "MB-System.content" block saying what that file holds. Set it once, before
// writing; it is not meant to change while files are being written. Without it,
// GLBs are written without metadata, as before.
void set_glb_run_metadata(const MetaValue &run_metadata);
void clear_glb_run_metadata();

[[nodiscard]] bool write_mesh_glb_file(
    const std::filesystem::path &path,
    const Mesh &mesh,
    std::string *error,
    const GlbContent &content = GlbContent());

[[nodiscard]] bool write_pointcloud_glb_file(
    const std::filesystem::path &path,
    const PointCloud &pointcloud,
    std::string *error,
    const GlbContent &content = GlbContent());

[[nodiscard]] bool write_normal_lines_glb_file(
    const std::filesystem::path &path,
    const OrientedPointCloud &pointcloud,
    double line_length,
    std::string *error,
    const GlbContent &content = GlbContent());

// Draws polylines, for example the sensor track, as line segments. Positions are
// written as given, so they must already be in the output frame.
[[nodiscard]] bool write_track_glb_file(
    const std::filesystem::path &path,
    const std::vector<std::vector<Vec3>> &polylines,
    std::string *error,
    const GlbContent &content = GlbContent());

[[nodiscard]] bool write_origin_ray_lines_glb_file(
    const std::filesystem::path &path,
    const CollectedPointCloud &pointcloud,
    double line_length,
    std::string *error,
    const GlbContent &content = GlbContent());
