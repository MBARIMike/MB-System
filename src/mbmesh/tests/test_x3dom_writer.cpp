#include "io/x3dom_writer.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

bool check(bool condition, const std::string &message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message.c_str());
    }
    return condition;
}

bool contains(const std::string &text, const std::string &part) {
    return text.find(part) != std::string::npos;
}

std::string write_page(const X3DomWriterOptions &options) {
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "mbmesh_test_x3dom.html";
    std::string error;
    if (!write_glb_x3dom_file(path, options, &error)) {
        std::fprintf(stderr, "FAIL: writing the page: %s\n", error.c_str());
        return "";
    }
    std::ifstream input(path);
    std::string page((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    std::filesystem::remove(path);
    return page;
}

X3DomModel model(const std::string &uri, const std::string &label, const std::string &group, bool visible) {
    X3DomModel m;
    m.uri = uri;
    m.label = label;
    m.group = group;
    m.visible = visible;
    return m;
}

void touch(const std::filesystem::path &path) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream(path) << "x";
}

} // namespace

int main() {
    bool ok = true;

    X3DomGeoOrigin geo;
    geo.origin_lat = 36.6;
    geo.origin_lon = -122.4;
    geo.origin_elevation = -10.0;
    geo.center_lat = 36.61;
    geo.center_lon = -122.39;
    geo.mean_elevation = -50.0;
    geo.view_altitude = 100.0;

    // ---- the page has the controls and the processing log of mbgrd2gltf's page
    {
        X3DomWriterOptions options;
        options.title = "OceanImaging2018/20180923d1/multibeam";
        options.geo = geo;
        options.models = {model("mbmesh_output/mesh.glb", "mesh.glb", "mbmesh_output", true)};
        options.log_messages = {"INFO first message", "DEBUG second message"};
        const std::string page = write_page(options);

        ok &= check(contains(page, "https://www.x3dom.org/x3dom/dist/x3dom-full.js"), "the page loads X3DOM");
        ok &= check(contains(page, "Navigation Mode:") && contains(page, "</input> Stats") && contains(page, "</input> Log"),
                    "the page has the navigation mode and debug display controls");
        ok &= check(contains(page, "runtime.showAll()") && contains(page, "runtime.resetView()"), "the page has Show Everything and Reset View");
        ok &= check(contains(page, "id='model-title'>OceanImaging2018/20180923d1/multibeam<") &&
                        contains(page, "<title>OceanImaging2018/20180923d1/multibeam</title>"),
                    "the title is the survey directories");
        ok &= check(contains(page, "Show Processing Log") && contains(page, "id='log-panel'") && contains(page, "function toggleLogPanel()"),
                    "the page has the processing log panel and its button");
        ok &= check(contains(page, "Processing Log (2 messages)") &&
                        contains(page, "<div class='log-line'>INFO first message</div>") &&
                        contains(page, "<div class='log-line'>DEBUG second message</div>"),
                    "the log shows each message and their number");

        // No GeoOrigin: no GeoOrigin node, and the viewpoint does not refer to one.
        ok &= check(!contains(page, "<GeoOrigin") && !contains(page, "geoOrigin='GRID_ORIGIN'"), "without a GeoOrigin the page declares none");
        ok &= check(contains(page, "<GeoViewpoint DEF='OVERVIEW' position='36.610000 -122.390000 50.0'"),
                    "the viewpoint is above the center of the data (latitude, longitude, elevation)");
    }

    // ---- a check box for every model; models that start switched off load when first wanted
    {
        X3DomWriterOptions options;
        options.title = "survey";
        options.geo = geo;
        options.models = {
            model("mbmesh_output/mesh.glb", "mesh.glb", "mbmesh_output", true),
            model("mbmesh_output/track.glb", "track.glb", "mbmesh_output", true),
            model("mbmesh_output/pointcloud.glb", "pointcloud.glb", "mbmesh_output", false),
            model("ZTopo.glb", "ZTopo.glb", "Other GLBs", false),
        };
        options.models[3].group_note = "Only if they use the same GeoOrigin & <so>.";
        const std::string page = write_page(options);

        ok &= check(contains(page, "id='model-checkbox-0' type='checkbox' checked onclick='toggleModel(0, this.checked);'") &&
                        contains(page, "id='model-checkbox-1' type='checkbox' checked onclick='toggleModel(1, this.checked);'"),
                    "mesh and track start checked");
        ok &= check(contains(page, "id='model-checkbox-2' type='checkbox' onclick='toggleModel(2, this.checked);'") &&
                        contains(page, "id='model-checkbox-3' type='checkbox' onclick='toggleModel(3, this.checked);'"),
                    "the other models start unchecked");
        ok &= check(contains(page, "<b>mbmesh_output</b>") && contains(page, "<b>Other GLBs</b>"), "models are listed under a heading for each group");
        ok &= check(page.find("<b>mbmesh_output</b>") == page.rfind("<b>mbmesh_output</b>"), "a group heading is written once");
        ok &= check(contains(page, "font-size:smaller;'>Only if they use the same GeoOrigin &amp; &lt;so&gt;.</td>"),
                    "a group note is written, escaped, under its heading");

        ok &= check(contains(page, "<switch id='switch_0' whichChoice='0'>") && contains(page, "<switch id='switch_2' whichChoice='-1'>"),
                    "each model has a switch, on or off at the start");
        ok &= check(contains(page, "<Inline id='inline_0' url='mbmesh_output/mesh.glb' nameSpaceName='model_0'") &&
                        contains(page, "<Inline id='inline_1' url='mbmesh_output/track.glb' nameSpaceName='model_1'"),
                    "models that start visible are loaded with their own namespace");
        ok &= check(!contains(page, "<Inline id='inline_2'") && !contains(page, "<Inline id='inline_3'"),
                    "models that start switched off are not loaded until they are wanted");
        ok &= check(contains(page, "{ uri: 'mbmesh_output/pointcloud.glb', loaded: false }") &&
                        contains(page, "{ uri: 'mbmesh_output/mesh.glb', loaded: true }") && contains(page, "function toggleModel(index, on)"),
                    "the page knows each model's URL and whether it is loaded");
    }

    // ---- with a GeoOrigin
    {
        X3DomWriterOptions options;
        options.use_geoorigin = true;
        options.geo = geo;
        options.models = {model("mbmesh_output/mesh.glb", "mesh.glb", "mbmesh_output", true)};
        const std::string page = write_page(options);

        ok &= check(contains(page, "<GeoOrigin DEF='GRID_ORIGIN' geoSystem='\"GD\" \"WE\"' geoCoords='36.600000 -122.400000 -10.00'>"),
                    "the GeoOrigin is declared, latitude first, as mbgrd2gltf does");
        ok &= check(contains(page, "geoOrigin='GRID_ORIGIN'"), "the viewpoint refers to the GeoOrigin");
    }

    // ---- log messages and labels are escaped, and the page says when some were left out
    {
        X3DomWriterOptions options;
        options.geo = geo;
        options.models = {model("a.glb", "<b>label</b> & more", "g", true)};
        options.log_messages = {"a < b & c > d \"q\" 'x' <script>alert(1)</script>"};
        options.log_messages_omitted = 7;
        const std::string page = write_page(options);

        ok &= check(contains(page, "a &lt; b &amp; c &gt; d &quot;q&quot; &#39;x&#39; &lt;script&gt;alert(1)&lt;/script&gt;"),
                    "log messages are escaped");
        ok &= check(!contains(page, "<script>alert(1)") && !contains(page, "<b>label</b>"), "nothing from a message or a label can add markup to the page");
        ok &= check(contains(page, "Processing Log (8 messages)") && contains(page, "7 further messages are not shown"),
                    "the page counts and mentions the messages that were left out");
    }

    // ---- errors
    {
        X3DomWriterOptions one;
        one.models = {model("a.glb", "a.glb", "g", true)};
        std::string error;
        ok &= check(!write_glb_x3dom_file("", one, &error) && !error.empty(), "an empty path is an error");
        ok &= check(!write_glb_x3dom_file(std::filesystem::temp_directory_path() / "x.html", X3DomWriterOptions(), &error) && !error.empty(),
                    "a page with no models is an error");
        ok &= check(!write_glb_x3dom_file("/nonexistent_directory/page.html", one, &error) && !error.empty(), "an unwritable path is an error");
    }

    // ---- where the page goes, and what it offers
    {
        const std::filesystem::path base = std::filesystem::temp_directory_path() / "mbmesh_test_viewer";
        std::filesystem::remove_all(base);
        const std::filesystem::path survey = base / "OceanImaging2018" / "20180923d1" / "multibeam";
        const std::filesystem::path output = survey / "mbmesh_output";
        touch(output / "mesh.glb");
        touch(output / "track.glb");
        touch(output / "pointcloud.glb");
        touch(output / "mesh.html");                     // not a GLB
        touch(output / "tiles" / "tile_0_0.glb");        // repeats mesh.glb, so not listed
        touch(survey / "ZTopo.glb");
        touch(survey / "my file.glb");
        touch(survey / "notes.txt");

        ok &= check(viewer_web_root(output) == survey, "the web root is the parent of the output directory");
        ok &= check(viewer_page_path(output) == survey / "mbmesh_output.html", "the page is named after the output directory");
        ok &= check(viewer_web_root(output.string() + "/") == survey && viewer_page_path(output.string() + "/") == survey / "mbmesh_output.html",
                    "a trailing slash on the output directory makes no difference");
        ok &= check(viewer_title(survey) == "OceanImaging2018/20180923d1/multibeam", "the title is the last three directories of the web root");
        ok &= check(viewer_title("/a/b") == "a/b" && viewer_title("/a") == "a", "a shorter path gives a shorter title");

        const std::vector<X3DomModel> models = find_viewer_models(survey, output);
        std::vector<std::string> uris;
        for (const X3DomModel &m : models) {
            uris.push_back(m.uri);
        }
        const std::vector<std::string> expected = {"mbmesh_output/mesh.glb", "mbmesh_output/track.glb", "mbmesh_output/pointcloud.glb",
                                                   "ZTopo.glb", "my%20file.glb"};
        ok &= check(uris == expected, "the models are mesh, track, the rest of the output directory, then the GLBs beside it, with spaces encoded");
        ok &= check(models.size() == 5 && models[0].visible && models[1].visible && !models[2].visible && !models[3].visible && !models[4].visible,
                    "only mesh and track start visible");
        ok &= check(models.size() == 5 && models[0].group == "mbmesh_output" && models[3].group != "mbmesh_output" && !models[3].group_note.empty() && models[3].label == "ZTopo.glb" &&
                        models[4].label == "my file.glb",
                    "the groups and labels tell the output directory's GLBs from the others");

        // Without mesh.glb nothing starts visible; no GLBs beside it if the output is not in the web root's top level
        std::filesystem::remove(output / "mesh.glb");
        std::filesystem::remove(output / "track.glb");
        const std::vector<X3DomModel> without_mesh = find_viewer_models(survey, output);
        ok &= check(!without_mesh.empty() && without_mesh[0].uri == "mbmesh_output/pointcloud.glb" && !without_mesh[0].visible,
                    "only models that exist are offered");

        std::filesystem::remove_all(base);
    }

    std::printf(ok ? "x3dom writer tests passed\n" : "x3dom writer tests FAILED\n");
    return ok ? 0 : 1;
}
