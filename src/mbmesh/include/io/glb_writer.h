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

// How the points of a point cloud are colored by height: with the Haxby color table that mbm_grdplot uses by default, 11
// colors from dark blue to white, changing linearly between 11 heights. HaxbyLinear is "mbm_grdplot -G5": the heights
// are equally spaced, at rounded values chosen from the range. HaxbyEqualized is "mbm_grdplot -G5 -S": the heights
// divide the points into cells of equal numbers. None is one orange.
enum class PointColorMap { None, HaxbyLinear, HaxbyEqualized };

// With heights, one value for each point, and a colormap other than None, the points get a COLOR_0 attribute that
// colors them by it (see colors_for_values in glb_writer.cpp), and the heights the color changes between are added to
// the content description. Without heights, or with None, the points are one orange.
// The names --pointcloud-colormap takes, "linear, equalized, none", and the parsing of one.
[[nodiscard]] const char *point_color_map_names();
[[nodiscard]] const char *point_color_map_name(PointColorMap colormap);
[[nodiscard]] bool parse_point_color_map(const std::string &name, PointColorMap *colormap);

[[nodiscard]] bool write_pointcloud_glb_file(
    const std::filesystem::path &path,
    const PointCloud &pointcloud,
    std::string *error,
    const GlbContent &content = GlbContent(),
    const std::vector<double> *heights = nullptr,
    PointColorMap colormap = PointColorMap::HaxbyLinear);

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
