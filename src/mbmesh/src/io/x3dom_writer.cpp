#include "io/x3dom_writer.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace {

void set_error(std::string *error, const std::string &message) {
    if (error != nullptr) {
        *error = message;
    }
}

[[nodiscard]] std::string escape_html(const std::string &text) {
    std::string escaped;
    escaped.reserve(text.size());

    for (const char character : text) {
        switch (character) {
        case '&':
            escaped += "&amp;";
            break;
        case '<':
            escaped += "&lt;";
            break;
        case '>':
            escaped += "&gt;";
            break;
        case '"':
            escaped += "&quot;";
            break;
        case '\'':
            escaped += "&#39;";
            break;
        default:
            escaped += character;
            break;
        }
    }

    return escaped;
}

// An absolute path with "." and ".." resolved and no trailing separator.
[[nodiscard]] std::filesystem::path normalized_directory(const std::filesystem::path &directory) {
    std::error_code filesystem_error;
    std::filesystem::path absolute = std::filesystem::absolute(directory, filesystem_error);
    if (filesystem_error) {
        absolute = directory;
    }
    absolute = absolute.lexically_normal();
    if (!absolute.has_filename() && absolute.has_parent_path() && absolute != absolute.root_path()) {
        absolute = absolute.parent_path();   // "/a/b/" -> "/a/b"
    }
    return absolute;
}

// Percent-encodes what a URL path cannot hold as it is. Slashes are kept.
[[nodiscard]] std::string url_encode(const std::string &text) {
    std::string encoded;
    for (const unsigned char character : text) {
        if (std::isalnum(character) || character == '-' || character == '.' || character == '_' || character == '~' || character == '/') {
            encoded += static_cast<char>(character);
        } else {
            std::ostringstream hex;
            hex << '%' << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(character);
            encoded += hex.str();
        }
    }
    return encoded;
}

[[nodiscard]] bool is_glb(const std::filesystem::directory_entry &entry) {
    if (!entry.is_regular_file()) {
        return false;
    }
    std::string extension = entry.path().extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) { return std::tolower(c); });
    return extension == ".glb";
}

// The names of the .glb files directly in a directory, sorted.
[[nodiscard]] std::vector<std::string> glb_names(const std::filesystem::path &directory) {
    std::vector<std::string> names;
    std::error_code filesystem_error;
    for (std::filesystem::directory_iterator it(directory, filesystem_error), end; !filesystem_error && it != end; it.increment(filesystem_error)) {
        if (is_glb(*it)) {
            names.push_back(it->path().filename().string());
        }
    }
    std::sort(names.begin(), names.end());
    return names;
}

constexpr std::size_t maximum_viewer_models = 200;

// The reference the root MetadataSet cites: the document that describes the names.
constexpr const char *metadata_reference = "https://github.com/dwcaress/MB-System/blob/master/src/mbmesh/GEOREFERENCE.md";

// An X3D MFString: each string in double quotes, with \ and " escaped.
[[nodiscard]] std::string mf_string(const std::vector<std::string> &strings) {
    std::string result;
    for (const std::string &text : strings) {
        std::string quoted = "\"";
        for (const char character : text) {
            if (character == '"' || character == '\\') {
                quoted += '\\';
            }
            quoted += character;
        }
        quoted += '"';
        result += (result.empty() ? "" : " ") + quoted;
    }
    return result;
}

[[nodiscard]] bool all_of_type(const MetaValue &array, MetaValue::Type type) {
    for (const MetaValue &item : array.items) {
        if (item.type != type) {
            return false;
        }
    }
    return !array.items.empty();
}

// Writes a value as X3D metadata nodes: an object as a MetadataSet, a string as a
// MetadataString, a number as a MetadataDouble, a boolean as a MetadataBoolean,
// and an array of numbers or strings as one node holding all of them.
void write_metadata_node(std::ostream &out, const MetaValue &value, const std::string &name, int depth, const std::string &extra_attributes = "") {
    const std::string indent(static_cast<std::size_t>(depth) * 2, ' ');
    const std::string head = "name='" + escape_html(name) + "'" + extra_attributes;

    switch (value.type) {
    case MetaValue::Type::String:
        out << indent << "<MetadataString " << head << " value='" << escape_html(mf_string({value.text})) << "'></MetadataString>\n";
        break;
    case MetaValue::Type::Number:
        out << indent << "<MetadataDouble " << head << " value='" << metadata_number_text(value.number) << "'></MetadataDouble>\n";
        break;
    case MetaValue::Type::Bool:
        out << indent << "<MetadataBoolean " << head << " value='" << (value.flag ? "true" : "false") << "'></MetadataBoolean>\n";
        break;
    case MetaValue::Type::Array:
        if (all_of_type(value, MetaValue::Type::Number)) {
            std::string numbers;
            for (const MetaValue &item : value.items) {
                numbers += (numbers.empty() ? "" : " ") + metadata_number_text(item.number);
            }
            out << indent << "<MetadataDouble " << head << " value='" << numbers << "'></MetadataDouble>\n";
        } else if (all_of_type(value, MetaValue::Type::String)) {
            std::vector<std::string> strings;
            for (const MetaValue &item : value.items) {
                strings.push_back(item.text);
            }
            out << indent << "<MetadataString " << head << " value='" << escape_html(mf_string(strings)) << "'></MetadataString>\n";
        } else {
            out << indent << "<MetadataSet " << head << ">\n";
            for (std::size_t i = 0; i < value.items.size(); i++) {
                write_metadata_node(out, value.items[i], std::to_string(i), depth + 1);
            }
            out << indent << "</MetadataSet>\n";
        }
        break;
    case MetaValue::Type::Object:
        out << indent << "<MetadataSet " << head << ">\n";
        for (const auto &member : value.members) {
            if (!member.first.empty() && member.first[0] == '@') {
                continue;   // JSON-LD keywords are for the GLB; the page shows the values
            }
            write_metadata_node(out, member.second, member.first, depth + 1);
        }
        out << indent << "</MetadataSet>\n";
        break;
    }
}

// The style, the same as the page mbgrd2gltf writes.
constexpr const char *page_style = R"CSS(
      .x3dom-logContainer { bottom: 0px; position: absolute; }
      #log-panel {
        position: fixed;
        bottom: 0;
        left: 0;
        right: 0;
        max-height: 30vh;
        background-color: rgba(240,240,240,0.95);
        border-top: 2px solid #666;
        display: none;
        flex-direction: column;
        z-index: 2000;
      }
      #log-panel.visible { display: flex; }
      #log-header {
        background-color: #666;
        color: white;
        padding: 8px 12px;
        font-weight: bold;
        display: flex;
        justify-content: space-between;
        align-items: center;
        cursor: pointer;
      }
      #log-content {
        flex: 1;
        overflow-y: auto;
        padding: 8px;
        font-family: 'Courier New', monospace;
        font-size: 12px;
      }
      .log-line {
        padding: 2px 0;
        white-space: pre-wrap;
        word-wrap: break-word;
      }
      #toggle-log-btn {
        position: fixed;
        bottom: 10px;
        left: 10px;
        background-color: rgba(102,102,102,0.9);
        color: white;
        border: none;
        padding: 10px 15px;
        border-radius: 4px;
        cursor: pointer;
        z-index: 1500;
        font-weight: bold;
      }
      #toggle-log-btn:hover { background-color: rgba(85,85,85,0.9); }
      #model-title {
        position: absolute;
        top: 0px;
        left: 50%;
        transform: translateX(-50%);
        background-color: rgba(199,202,204,.7);
        padding: 4px 20px;
        margin: 2px;
        margin-top: 2px;
        z-index: 1000;
        font-size: 16px;
        font-weight: bold;
        white-space: nowrap;
      }
)CSS";

constexpr const char *page_script = R"JS(
      // Show or hide a model. A model that was not shown at the start is loaded the
      // first time it is switched on, so large files cost nothing until they are wanted.
      function toggleModel(index, on) {
        var model = viewerModels[index];
        if (on && !model.loaded) {
          var inline = document.createElement('Inline');
          inline.setAttribute('id', 'inline_' + index);
          inline.setAttribute('url', model.uri);
          inline.setAttribute('nameSpaceName', 'model_' + index);
          inline.setAttribute('mapDEFToID', 'true');
          document.getElementById('transform_' + index).appendChild(inline);
          model.loaded = true;
        }
        document.getElementById('switch_' + index).setAttribute('whichChoice', on ? 0 : -1);
      }

      function toggleLogPanel() {
        var panel = document.getElementById('log-panel');
        var btn = document.getElementById('toggle-log-btn');
        if (panel.classList.contains('visible')) {
          panel.classList.remove('visible');
          btn.textContent = 'Show Processing Log';
        } else {
          panel.classList.add('visible');
          btn.textContent = 'Hide Processing Log';
        }
      }
)JS";

} // namespace

std::filesystem::path viewer_web_root(const std::filesystem::path &output_directory) {
    return normalized_directory(output_directory).parent_path();
}

std::filesystem::path viewer_page_path(const std::filesystem::path &output_directory) {
    const std::filesystem::path directory = normalized_directory(output_directory);
    return directory.parent_path() / (directory.filename().string() + ".html");
}

std::string viewer_title(const std::filesystem::path &web_root) {
    const std::filesystem::path directory = normalized_directory(web_root);
    std::vector<std::string> names;
    for (const std::filesystem::path &part : directory) {
        const std::string name = part.string();
        if (!name.empty() && name != "/" && name != "\\") {
            names.push_back(name);
        }
    }
    const std::size_t first = names.size() > 3 ? names.size() - 3 : 0;
    std::string title;
    for (std::size_t i = first; i < names.size(); i++) {
        title += (title.empty() ? "" : "/") + names[i];
    }
    return title;
}

std::vector<X3DomModel> find_viewer_models(const std::filesystem::path &web_root, const std::filesystem::path &output_directory) {
    std::vector<X3DomModel> models;
    const std::filesystem::path root = normalized_directory(web_root);
    const std::filesystem::path directory = normalized_directory(output_directory);
    const std::string directory_name = directory.filename().string();

    // The output directory: mesh.glb, then track.glb, then the rest by name.
    std::vector<std::string> names = glb_names(directory);
    std::stable_sort(names.begin(), names.end(), [](const std::string &a, const std::string &b) {
        const auto rank = [](const std::string &name) { return name == "mesh.glb" ? 0 : (name == "track.glb" ? 1 : 2); };
        return rank(a) < rank(b);
    });
    for (const std::string &name : names) {
        X3DomModel model;
        model.uri = url_encode(directory_name + "/" + name);
        model.label = name;
        model.group = directory_name;
        model.visible = name == "mesh.glb" || name == "track.glb";
        models.push_back(model);
    }

    // The web root: other GLBs, such as the product of mbgrd2gltf.
    if (directory.parent_path() == root) {
        for (const std::string &name : glb_names(root)) {
            X3DomModel model;
            model.uri = url_encode(name);
            model.label = name;
            model.group = "Other GLBs in this directory";
            model.group_note = "They line up with this run only if they use the same GeoOrigin.";
            model.visible = false;
            models.push_back(model);
        }
    }

    if (models.size() > maximum_viewer_models) {
        models.resize(maximum_viewer_models);
    }
    return models;
}

bool write_glb_x3dom_file(const std::filesystem::path &html_path, const X3DomWriterOptions &options, std::string *error) {
    if (error != nullptr) {
        error->clear();
    }

    if (html_path.empty()) {
        set_error(error, "X3DOM output path is empty");
        return false;
    }

    if (options.models.empty()) {
        set_error(error, "X3DOM page has no models to show");
        return false;
    }

    std::ofstream output(html_path);
    if (!output) {
        set_error(error, "Unable to create X3DOM file: " + html_path.string());
        return false;
    }

    const std::string title = options.title.empty() ? options.models.front().uri : options.title;

    // GeoViewpoint position and centre of rotation, as latitude, longitude, elevation
    // (the X3D geospatial order), looking straight down with north up.
    const X3DomGeoOrigin &geo = options.geo;
    std::ostringstream position, center, origin_coordinates;
    position << std::fixed << std::setprecision(6) << geo.center_lat << " " << geo.center_lon << " " << std::setprecision(1)
             << (geo.mean_elevation + geo.view_altitude);
    center << std::fixed << std::setprecision(6) << geo.center_lat << " " << geo.center_lon << " " << std::setprecision(2) << geo.mean_elevation;
    origin_coordinates << std::fixed << std::setprecision(6) << geo.origin_lat << " " << geo.origin_lon << " " << std::setprecision(2)
                       << geo.origin_elevation;

    output << "<!DOCTYPE html>\n"
           << "<html style='width:100%; height:100%; border:0; margin:0; padding:0;'>\n"
           << "  <head>\n"
           << "    <meta http-equiv='X-UA-Compatible' content='chrome=1'></meta>\n"
           << "    <meta http-equiv='Content-Type' content='text/html;charset=utf-8'></meta>\n"
           << "    <title>" << escape_html(title) << "</title>\n"
           << "    <link type='text/css' href='https://www.x3dom.org/x3dom/dist/x3dom.css' rel='stylesheet'>\n"
           << "    <script type='text/javascript' src='https://www.x3dom.org/x3dom/dist/x3dom-full.js'></script>\n"
           << "    <style>" << page_style << "    </style>\n"
           << "  </head>\n"
           << "  <body style='width:100%; height:100%; border:0; margin:0; padding:0;'>\n"
           << "    <div id='HUDs_Div'>\n"
           << "      <div id='X3DOM_MBSystem_Links' class='group' style='margin:2px; margin-top:26px; padding:4px; "
              "background-color:rgba(199,202,204,.7); position:absolute; float:center; z-index:1000;'>\n"
           << "        <a href='https://www.x3dom.org'>X3DOM</a> output created with\n"
           << "        <a href='https://www.mbari.org/products/research-software/mb-system/'>MB-System</a> mbmesh.\n"
           << "      </div>\n"
           << "      <div id='model-title'>" << escape_html(title) << "</div>\n"
           << "      <div id='Interaction_Toolbox' style='margin:2px; padding:4px; padding-right:150px; "
              "background-color:rgba(199,202,204,.7);position:absolute; z-index:1000; right:0px; top:0px;'>\n"
           << "        <table>\n"
           << "          <tr>\n"
           << "            <td>Navigation Mode:\n"
           << "            </td>\n"
           << "            <td align='right'>\n"
           << "              <select style='float:right;' onchange='if (this.selectedIndex !== undefined) { "
              "var e = document.getElementById(&apos;x3dElement&apos;); "
              "if (this.options[this.selectedIndex].value === &apos;examine&apos;) { e.runtime.examine(); } "
              "else if (this.options[this.selectedIndex].value === &apos;lookat&apos;) { e.runtime.lookAt(); } "
              "else if (this.options[this.selectedIndex].value === &apos;walk&apos;) { e.runtime.walk(); } "
              "else if (this.options[this.selectedIndex].value === &apos;fly&apos;) { e.runtime.fly(); } "
              "else if (this.options[this.selectedIndex].value === &apos;helicopter&apos;) { e.runtime.helicopter(); } "
              "else if (this.options[this.selectedIndex].value === &apos;none&apos;) { e.runtime.noNav(); } }'>\n"
           << "                <option value='examine'>Examine\n                </option>\n"
           << "                <option value='lookat'>LookAt\n                </option>\n"
           << "                <option value='walk'>Walk\n                </option>\n"
           << "                <option value='fly'>Fly\n                </option>\n"
           << "                <option value='helicopter'>Helicopter\n                </option>\n"
           << "                <option value='none'>None\n                </option>\n"
           << "              </select>\n"
           << "            </td>\n"
           << "          </tr>\n"
           << "          <tr>\n"
           << "            <td>Debug Display:\n"
           << "            </td>\n"
           << "            <td align='right'>\n"
           << "              <input type='checkbox' onclick='if (this.checked) { "
              "document.getElementById(&apos;x3dElement&apos;).runtime.statistics(true); } else { "
              "document.getElementById(&apos;x3dElement&apos;).runtime.statistics(false); }'>\n"
           << "              </input> Stats\n"
           << "              <input type='checkbox' onclick='document.getElementById(&apos;x3dElement&apos;).runtime.debug();'>\n"
           << "              </input> Log\n"
           << "            </td>\n"
           << "          </tr>\n";

    // A check box for each model, under a heading for each group.
    std::string current_group;
    for (std::size_t i = 0; i < options.models.size(); i++) {
        const X3DomModel &model = options.models[i];
        if (i == 0 || model.group != current_group) {
            current_group = model.group;
            output << "          <tr>\n"
                   << "            <td colspan='2'><b>" << escape_html(current_group) << "</b></td>\n"
                   << "          </tr>\n";
            if (!model.group_note.empty()) {
                output << "          <tr>\n"
                       << "            <td colspan='2' style='max-width:18em; font-size:smaller;'>" << escape_html(model.group_note) << "</td>\n"
                       << "          </tr>\n";
            }
        }
        output << "          <tr>\n"
               << "            <td colspan='2'>\n"
               << "              <input id='model-checkbox-" << i << "' type='checkbox'" << (model.visible ? " checked" : "")
               << " onclick='toggleModel(" << i << ", this.checked);'>\n"
               << "              </input> " << escape_html(model.label) << "\n"
               << "            </td>\n"
               << "          </tr>\n";
    }

    output << "          <tr>\n"
           << "            <td>\n"
           << "              <button onclick='document.getElementById(&apos;x3dElement&apos;).runtime.showAll();'> Show Everything\n"
           << "              </button>\n"
           << "            </td>\n"
           << "            <td>\n"
           << "              <button onclick='document.getElementById(&apos;x3dElement&apos;).runtime.resetView();'> Reset View\n"
           << "              </button>\n"
           << "            </td>\n"
           << "          </tr>\n"
           << "        </table>\n"
           << "      </div>\n"
           << "    </div>\n"
           << "    <x3d id='x3dElement' showStat='false' showLog='false' style='width:100%; height:100%; border:0; margin:0; padding:0;'>\n"
           << "      <scene DEF='scene'>\n";

    if (options.use_geoorigin) {
        output << "        <GeoOrigin DEF='GRID_ORIGIN' geoSystem='\"GD\" \"WE\"' geoCoords='" << origin_coordinates.str() << "'></GeoOrigin>\n";
    }

    output << "        <GeoViewpoint DEF='OVERVIEW' position='" << position.str() << "' orientation='1 0 0 -1.5708' fieldOfView='0.785398' "
           << "centerOfRotation='" << center.str() << "'" << (options.use_geoorigin ? " geoOrigin='GRID_ORIGIN'" : "")
           << " geoSystem='\"GD\" \"WE\"'></GeoViewpoint>\n";

    // Documentation of the scene, from the same metadata that is in each GLB. The
    // geospatial facts go in the GeoMetadata summary, which takes only the keywords
    // X3D defines; everything else is a MetadataSet in its metadata field. WorldInfo
    // repeats the entries as readable lines, which X3DOM shows in its log.
    if (!options.metadata.members.empty()) {
        std::vector<std::string> lines;
        for (const auto &entry : flatten_metadata(options.metadata)) {
            lines.push_back(entry.first + ": " + entry.second);
        }
        output << "        <WorldInfo title='" << escape_html(title) << "' info='" << escape_html(mf_string(lines)) << "'></WorldInfo>\n"
               << "        <GeoMetadata DEF='GEO_METADATA' summary='" << escape_html(mf_string(options.geo_summary)) << "'>\n";
        write_metadata_node(output, options.metadata, "extras", 5, " containerField='metadata' reference='" + std::string(metadata_reference) + "'");
        output << "        </GeoMetadata>\n";
    }

    for (std::size_t i = 0; i < options.models.size(); i++) {
        const X3DomModel &model = options.models[i];
        output << "        <switch id='switch_" << i << "' whichChoice='" << (model.visible ? 0 : -1) << "'>\n"
               << "          <transform id='transform_" << i << "'>\n";
        if (model.visible) {
            output << "            <Inline id='inline_" << i << "' url='" << escape_html(model.uri) << "' nameSpaceName='model_" << i
                   << "' mapDEFToID='true'></Inline>\n";
        }
        output << "          </transform>\n"
               << "        </switch>\n";
    }

    output << "      </scene>\n"
           << "    </x3d>\n"
           << "    \n"
           << "    <button id='toggle-log-btn' onclick='toggleLogPanel()'>Show Processing Log</button>\n"
           << "    \n"
           << "    <div id='log-panel'>\n"
           << "      <div id='log-header' onclick='toggleLogPanel()'>\n"
           << "        <span>Processing Log (" << options.log_messages.size() + options.log_messages_omitted << " messages)</span>\n"
           << "        <span id='close-log'>&#10005;</span>\n"
           << "      </div>\n"
           << "      <div id='log-content'>\n";

    for (const std::string &message : options.log_messages) {
        output << "        <div class='log-line'>" << escape_html(message) << "</div>\n";
    }
    if (options.log_messages_omitted > 0) {
        output << "        <div class='log-line'>... " << options.log_messages_omitted << " further messages are not shown</div>\n";
    }

    output << "      </div>\n"
           << "    </div>\n"
           << "    \n"
           << "    <script>\n"
           << "      var viewerModels = [\n";
    for (std::size_t i = 0; i < options.models.size(); i++) {
        output << "        { uri: '" << options.models[i].uri << "', loaded: " << (options.models[i].visible ? "true" : "false") << " }"
               << (i + 1 < options.models.size() ? "," : "") << "\n";
    }
    output << "      ];\n"
           << page_script << "    </script>\n"
           << "  </body>\n"
           << "</html>\n";

    output.close();
    if (!output) {
        set_error(error, "Error while writing X3DOM file: " + html_path.string());
        return false;
    }

    return true;
}
