#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "provenance.h"

// GeoOrigin and GeoViewpoint values, in the same form mbgrd2gltf writes them.
struct X3DomGeoOrigin {
    double origin_lat = 0.0;
    double origin_lon = 0.0;
    double origin_elevation = 0.0;
    double center_lat = 0.0;
    double center_lon = 0.0;
    double mean_elevation = 0.0;
    double view_altitude = 0.0;

    // The overview viewpoint in the coordinates of the models themselves (ECEF meters
    // minus the GeoOrigin's ECEF position, or plain ECEF without a GeoOrigin), set by
    // set_local_viewpoint(). X3DOM places a GeoViewpoint in absolute ECEF whatever its
    // geoOrigin says, which is millions of meters from models that are relative to
    // the GeoOrigin, so the page uses a plain Viewpoint in the models' coordinates
    // when this is set, and the GeoViewpoint above otherwise.
    bool has_local_view = false;
    double view_position[3] = {0.0, 0.0, 0.0};
    double view_center[3] = {0.0, 0.0, 0.0};
    double view_axis[3] = {0.0, 0.0, 1.0};   // orientation, as axis and angle
    double view_angle = 0.0;
};

// Sets the local viewpoint: above center_ecef, the ECEF position of the middle of
// the data, by view_altitude, looking straight down with north at the top of the
// view, in coordinates relative to origin_ecef (zeros if the models are absolute).
// Uses center_lat, center_lon and view_altitude of the structure.
void set_local_viewpoint(X3DomGeoOrigin *geo, const double center_ecef[3], const double origin_ecef[3]);

// One GLB the page can show, with a checkbox to switch it on and off.
struct X3DomModel {
    std::string uri;         // relative to the page, with characters that need it percent-encoded
    std::string label;       // shown beside the checkbox
    std::string group;       // models with the same group are listed together under that heading
    std::string group_note;  // small print under the heading, for the first model of a group; may be empty
    bool visible = false;    // shown when the page opens; others are loaded when first switched on
};

struct X3DomWriterOptions {
    // Shown at the top of the page and as its title.
    std::string title;

    // When set, the GLBs hold ECEF offsets from the GeoOrigin and the page
    // declares a GeoOrigin node for the GeoViewpoint to refer to, as mbgrd2gltf
    // does. The GeoViewpoint is written either way.
    bool use_geoorigin = false;
    X3DomGeoOrigin geo;

    // The GLBs the page can show.
    std::vector<X3DomModel> models;

    // The georeferencing and provenance metadata, written into the page as X3D
    // metadata: WorldInfo, and a GeoMetadata node whose summary is geo_summary and
    // whose metadata field holds this tree as a MetadataSet. The same tree is
    // written into each GLB as asset.extras. Not written if empty.
    MetaValue metadata;
    std::vector<std::string> geo_summary;   // keyword and value strings, an even number

    // The processing log shown in the page, one message per entry, and how many
    // further messages were left out of it.
    std::vector<std::string> log_messages;
    std::size_t log_messages_omitted = 0;
};

// Writes an X3DOM viewer page, in the style of the page mbgrd2gltf writes: a
// heads-up display with the navigation mode, debug display and view controls, a
// check box for each model, and a panel with the processing log.
[[nodiscard]] bool write_glb_x3dom_file(
    const std::filesystem::path &html_path,
    const X3DomWriterOptions &options,
    std::string *error);

// The viewer page lives in the parent of the output directory, named after the
// output directory, so that the parent is the root of the web server and GLBs
// that sit beside the output directory can be shown too:
//
//   <root>/mbmesh_output.html        <- the page
//   <root>/mbmesh_output/mesh.glb    <- the output directory
//   <root>/ZTopo.glb                 <- another GLB, for example from mbgrd2gltf
[[nodiscard]] std::filesystem::path viewer_web_root(const std::filesystem::path &output_directory);
[[nodiscard]] std::filesystem::path viewer_page_path(const std::filesystem::path &output_directory);

// The names of the last three directories of the web root, which identify the
// survey, for example "OceanImaging2018/20180923d1/multibeam".
[[nodiscard]] std::string viewer_title(const std::filesystem::path &web_root);

// The GLBs the page offers: those in the output directory, mesh.glb then
// track.glb then the rest by name, with mesh.glb and track.glb visible at the start,
// and then those in the web root, which start switched off. The tile files in the
// subdirectory of the output directory are not listed, since they repeat mesh.glb.
[[nodiscard]] std::vector<X3DomModel> find_viewer_models(
    const std::filesystem::path &web_root,
    const std::filesystem::path &output_directory);
