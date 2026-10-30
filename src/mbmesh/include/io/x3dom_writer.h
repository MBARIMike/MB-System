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

// A GeoViewpoint the user supplies with --geoviewpoint: where the camera is and where it looks, as
// WGS 84 geodetic coordinates, so that it does not depend on the GeoOrigin, the area or the grid, and can
// be used again in another run. The orientation, if given, is that of a GeoViewpoint: relative to the
// local frame at the position, as X3DOM takes it ('1 0 0 -1.5708' looks straight down with north at the
// top and east on the right, and no rotation looks level to the north). Heights are above the ellipsoid,
// like the GeoOrigin's elevation.
struct X3DomUserViewpoint {
    double longitude = 0.0;
    double latitude = 0.0;
    double elevation = 0.0;
    double center_longitude = 0.0;   // the center of rotation, and the point looked at without an orientation
    double center_latitude = 0.0;
    double center_elevation = 0.0;
    bool has_orientation = false;    // otherwise the camera looks from the position toward the center, with up at the top
    double axis[3] = {0.0, 0.0, 1.0};
    double angle = 0.0;
};

// Parses the value of --geoviewpoint, which is in the order of --geoorigin, longitude first: six numbers,
// lon,lat,elev,clon,clat,celev, the camera position and the point looked at; or ten,
// lon,lat,elev,ax,ay,az,angle,clon,clat,celev, with the orientation as an axis and an angle between the
// position and the center. Commas separate the numbers, with no spaces. The Copy GeoViewpoint button of the
// page gives the ten-number form. Returns false and says why in error.
[[nodiscard]] bool parse_user_viewpoint(const std::string &text, X3DomUserViewpoint *viewpoint, std::string *error);

// The orientation of a user viewpoint as a GeoViewpoint takes it, an axis and an angle relative to the local
// frame at the position: the one given, or, without one, the orientation that looks from the position toward
// the center with the top of the view toward local up, or toward north when looking straight up or down.
// Returns false if the position and center are the same point.
[[nodiscard]] bool geoviewpoint_orientation(const X3DomUserViewpoint &viewpoint, double axis[3], double *angle, std::string *error);

// One GLB the page can show, with a checkbox to switch it on and off.
struct X3DomModel {
    std::string uri;         // relative to the page, with characters that need it percent-encoded
    std::string label;       // shown beside the checkbox
    std::string group;       // models with the same group are listed together under that heading
    std::string group_note;  // small print under the heading, for the first model of a group; may be empty
    bool visible = false;    // shown when the page opens; others are loaded when first switched on
    // The uri is an OGC 3D Tiles tileset.json, shown with a GeoTileset node (X3DOM 1.8.4 and later builds that
    // have it), which is written into the page whether or not it is shown, with the GeoOrigin that the page
    // has, if any, as its child node.
    bool is_tileset = false;
};

struct X3DomWriterOptions {
    // Shown at the top of the page and as its title.
    std::string title;

    // When set, the GLBs hold ECEF offsets from the GeoOrigin and the page
    // declares a GeoOrigin node for the GeoViewpoint to refer to, as mbgrd2gltf
    // does. The GeoViewpoint is written either way.
    bool use_geoorigin = false;
    X3DomGeoOrigin geo;

    // GeoViewpoints the user supplied (--geoviewpoint), written after the two overview
    // viewpoints, as USER_GEOVIEW_1, USER_GEOVIEW_2 and so on, each with the GeoOrigin if there is one.
    std::vector<X3DomUserViewpoint> user_viewpoints;

    // If not zero, the page lifts everything this many meters above the ellipsoid, along the local vertical: each
    // model is in a GeoTransform with a translation up, and the heights of the viewpoints, the user's too, have it
    // added. It is for seafloor data at negative heights, which parts of X3DOM's GeoTileset do not handle. The Copy
    // GeoViewpoint button takes the lift off again, so its option is for the true heights.
    double lift_meters = 0.0;

    // The X3DOM script the page loads. A GeoTileset needs a build of X3DOM that has it (merged in October 2026),
    // which the build on www.x3dom.org, the default, does not.
    std::string x3dom_url = "https://www.x3dom.org/x3dom/dist/x3dom-full.js";

    // The GLBs the page can show.
    std::vector<X3DomModel> models;

    // The georeferencing and provenance metadata, written into the page as X3D
    // metadata: WorldInfo, and a GeoMetadata node whose summary is geo_summary and
    // whose metadata field holds this tree as a MetadataSet. The same tree is
    // written into each GLB as asset.extras. Not written if empty.
    MetaValue metadata;
    std::vector<std::string> geo_summary;   // keyword and value strings, an even number

    // List the models in accordions, one per directory level, instead of under flat headings: a model's group
    // is then a path of names separated by slashes, and each name is an accordion inside the one before.
    bool nested_groups = false;

    // The processing log shown in the page, one message per entry, and how many
    // further messages were left out of it.
    std::vector<std::string> log_messages;
    std::size_t log_messages_omitted = 0;

    // If set, the log is not written into the page, which a long run makes large and hard to edit. The page
    // loads this file instead, the first time the log panel is opened: a URL relative to the page, of a text
    // file with a message on each line, as write_log_file() writes it. The page must be served over HTTP, as
    // it must to load its GLBs. The messages above are then ignored.
    std::string log_url;
};

// Writes the log messages to a text file, a message on each line, and a last line saying how many further
// messages were left out, if any.
[[nodiscard]] bool write_log_file(
    const std::filesystem::path &log_path,
    const std::vector<std::string> &messages,
    std::size_t omitted,
    std::string *error);

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
//
// With page_in_output_directory the page is mbmesh.html inside the output directory, so the uris are
// relative to that directory and no GLBs from the web root are offered.
[[nodiscard]] std::vector<X3DomModel> find_viewer_models(
    const std::filesystem::path &web_root,
    const std::filesystem::path &output_directory,
    bool tileset = false,
    bool page_in_output_directory = false);

// The log of a run, <output>/mbmesh.log, which the pages load instead of holding it themselves.
[[nodiscard]] std::filesystem::path viewer_log_path(const std::filesystem::path &output_directory);

// The URL of that log from the page beside the output directory, for X3DomWriterOptions::log_url.
[[nodiscard]] std::string viewer_log_url(const std::filesystem::path &output_directory);

// The page inside the output directory, <output>/mbmesh.html, which shows the GLBs of the one run.
[[nodiscard]] std::filesystem::path viewer_inner_page_path(const std::filesystem::path &output_directory);

// The page that a directory input writes, mbmesh_survey.html in the current directory.
[[nodiscard]] std::filesystem::path survey_page_path();

// Every GLB beneath survey_directory, as models whose groups are the directory paths, starting with the
// name of the survey directory itself, with uris relative to page_directory. The tiles of a tileset (the
// directories "tiles" and "tileset") are left out, and the tileset.json of a tileset is offered in their
// place. mesh.glb comes first in a directory, then track.glb, then the rest by name. The track.glb files start switched on, the others off.
// From the georeference in the GLBs written by mbmesh, geo gets an overview as a run's page has: its center is
// the middle of the union of the extents, its height fits them in a 45 degree view. The GeoOrigin of geo is that
// of the first GLB with vertices relative to one, and use_geoorigin is set; with none, the models are absolute.
// If no GLB has a georeference, geo and use_geoorigin are left as they were.
//
// The GLBs are meant to be placed relative to one GeoOrigin. If geoorigin_mismatches is given, a message is added
// to it for each GLB whose GeoOrigin is not that of the first with relative vertices (its center differs by more than 1 cm), and for each
// whose vertices are absolute when there is a GeoOrigin, since such a GLB does not line up with the others.
[[nodiscard]] std::vector<X3DomModel> find_survey_models(
    const std::filesystem::path &survey_directory,
    const std::filesystem::path &page_directory,
    bool *use_geoorigin,
    X3DomGeoOrigin *geo,
    std::vector<std::string> *geoorigin_mismatches = nullptr);
