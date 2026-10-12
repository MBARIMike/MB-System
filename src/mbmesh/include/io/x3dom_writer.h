#pragma once

#include <filesystem>
#include <string>

// GeoOrigin and GeoViewpoint values, in the same form mbgrd2gltf writes them.
struct X3DomGeoOrigin {
    double origin_lat = 0.0;
    double origin_lon = 0.0;
    double origin_elevation = 0.0;
    double center_lat = 0.0;
    double center_lon = 0.0;
    double mean_elevation = 0.0;
    double view_altitude = 0.0;
};

struct X3DomWriterOptions {
    // Text displayed as the generated HTML document title.
    std::string title = "MB-System mbmesh Mesh Viewer";

    // When set, the GLB holds ECEF offsets from the GeoOrigin and the page
    // places it with GeoOrigin and GeoViewpoint nodes (as mbgrd2gltf does).
    bool use_geoorigin = false;
    X3DomGeoOrigin geo;
};

[[nodiscard]] bool write_glb_x3dom_file(
    const std::filesystem::path &html_path,
    const std::string &glb_uri,
    const X3DomWriterOptions &options,
    std::string *error);
