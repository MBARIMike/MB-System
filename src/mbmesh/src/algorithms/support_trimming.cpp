#include "algorithms/support_trimming.h"

#include "math/kdtree.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace {

constexpr std::size_t maximum_spacing_samples = 2048;
constexpr double automatic_support_radius_scale = 2.5;
constexpr double automatic_normal_offset_scale = 1.5;

// A vertex put back in a void is probed in eight directions in its tangent plane, and a sounding must be within this
// multiple of the distance of its nearest sounding of each probe; if the nearest is farther than the support radius,
// the median over its nearest few of the cosine of the angle between the normal of the mesh and the direction to the
// sounding may not exceed the second constant (see support_trimming()).
// A vertex short of neighbors is put back if the nearest sounding is within this multiple of the support radius.
constexpr double near_sounding_scale = 3.0;
constexpr double void_probe_tolerance = 0.5;
constexpr std::size_t void_neighbor_count = 5;
constexpr double void_maximum_normal_alignment = 0.8;

enum class VertexSupportResult {
    supported,
    insufficient_neighbors,
    excessive_normal_offset,
    insufficient_normal_alignment,
};

double compact_quadratic_weight(double distance_squared, double radius_squared) {
    if (radius_squared <= 0.0 || distance_squared >= radius_squared) {
        return 0.0;
    }

    const double normalized_distance_squared = distance_squared / radius_squared;
    const double falloff = 1.0 - normalized_distance_squared;
    return falloff * falloff;
}

std::vector<Vec3> sample_positions(const OrientedPointCloud &oriented_points) {
    std::vector<Vec3> positions;
    positions.reserve(oriented_points.size());
    for (const OrientedPoint &oriented_point : oriented_points) {
        positions.push_back(oriented_point.point);
    }
    return positions;
}

double estimate_point_spacing(const std::vector<Vec3> &points,
                              const KDTree &tree,
                              std::size_t neighbor_rank) {
    if (points.size() < 2) {
        return 0.0;
    }

    neighbor_rank = std::max<std::size_t>(1, neighbor_rank);
    const std::size_t spacing_sample_count =
        std::min(points.size(), maximum_spacing_samples);

    std::vector<double> spacing_samples;
    spacing_samples.reserve(spacing_sample_count);

    for (std::size_t sample = 0; sample < spacing_sample_count; sample++) {
        const std::size_t point_index = spacing_sample_count == 1 ? 0 : sample * (points.size() - 1) / (spacing_sample_count - 1);

        const std::vector<KDTree::Neighbor> neighbors = tree.k_nearest(points[point_index], neighbor_rank, point_index);
        if (neighbors.empty()) {
            continue;
        }

        const double distance_squared = neighbors.back().distance_squared;
        if (distance_squared > vec3_epsilon * vec3_epsilon) {
            spacing_samples.push_back(std::sqrt(distance_squared));
        }
    }

    if (spacing_samples.empty()) {
        return 0.0;
    }

    const std::size_t median_index = spacing_samples.size() / 2;
    std::nth_element(spacing_samples.begin(),
                     spacing_samples.begin() + median_index,
                     spacing_samples.end());
    return spacing_samples[median_index];
}

std::vector<Vec3> calculate_vertex_normals(const Mesh &mesh) {
    std::vector<Vec3> normals(mesh.vertices.size(), Vec3(0.0));

    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const unsigned int a = mesh.indices[i];
        const unsigned int b = mesh.indices[i + 1];
        const unsigned int c = mesh.indices[i + 2];
        if (a >= mesh.vertices.size() ||
            b >= mesh.vertices.size() ||
            c >= mesh.vertices.size()) {
            continue;
        }

        const Vec3 face_normal = cross(mesh.vertices[b] - mesh.vertices[a], mesh.vertices[c] - mesh.vertices[a]);
        if (face_normal.length_squared() <= vec3_epsilon * vec3_epsilon) {
            continue;
        }

        normals[a] += face_normal;
        normals[b] += face_normal;
        normals[c] += face_normal;
    }

    for (Vec3 &normal : normals) {
        if (normal.length_squared() > vec3_epsilon * vec3_epsilon) {
            normal = normalize(normal);
        } else {
            normal = Vec3(0.0, 0.0, 1.0);
        }
    }

    return normals;
}

VertexSupportResult vertex_support_result(
    const Vec3 &vertex,
    const Vec3 &mesh_normal,
    const OrientedPointCloud &oriented_points,
    const KDTree &tree,
    double support_radius,
    double max_normal_offset,
    std::size_t minimum_neighbors,
    double minimum_normal_alignment) {
    const double support_radius_squared = support_radius * support_radius;
    const std::vector<KDTree::Neighbor> neighbors = tree.radius_search_squared(vertex, support_radius_squared);

    Vec3 weighted_normal(0.0);
    double weighted_normal_offset = 0.0;
    double accumulated_weight = 0.0;
    std::size_t contributing_neighbors = 0;

    for (const KDTree::Neighbor &neighbor : neighbors) {
        const OrientedPoint &sample = oriented_points[neighbor.index];
        if (sample.normal.length_squared() <= vec3_epsilon * vec3_epsilon) {
            continue;
        }

        const double weight = compact_quadratic_weight(neighbor.distance_squared, support_radius_squared);
        if (weight <= vec3_epsilon) {
            continue;
        }

        const Vec3 sample_normal = normalize(sample.normal);
        weighted_normal += sample_normal * weight;
        weighted_normal_offset += std::fabs(dot(vertex - sample.point, sample_normal)) * weight;
        accumulated_weight += weight;
        contributing_neighbors++;
    }

    if (contributing_neighbors < minimum_neighbors ||
        accumulated_weight <= vec3_epsilon) {
        return VertexSupportResult::insufficient_neighbors;
    }

    const double mean_normal_offset = weighted_normal_offset / accumulated_weight;
    if (mean_normal_offset > max_normal_offset) {
        return VertexSupportResult::excessive_normal_offset;
    }

    if (minimum_normal_alignment > 0.0) {
        if (weighted_normal.length_squared() <= vec3_epsilon * vec3_epsilon ||
            mesh_normal.length_squared() <= vec3_epsilon * vec3_epsilon) {
            return VertexSupportResult::insufficient_normal_alignment;
        }

        const double alignment = std::fabs(dot(normalize(mesh_normal), normalize(weighted_normal)));
        if (alignment < minimum_normal_alignment) {
            return VertexSupportResult::insufficient_normal_alignment;
        }
    }

    return VertexSupportResult::supported;
}

// Vertices on an edge that only one triangle has: the raw mesh is open there.
std::vector<bool> open_boundary_vertices(const Mesh &raw_mesh) {
    const std::size_t vertex_count = raw_mesh.vertices.size();
    const std::size_t triangle_count = raw_mesh.indices.size() / 3;
    std::unordered_map<std::uint64_t, int> edge_uses;
    edge_uses.reserve(raw_mesh.indices.size());
    const auto edge_key = [](unsigned int a, unsigned int b) { return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b); };
    for (std::size_t t = 0; t < triangle_count; t++) {
        const unsigned int *tri = &raw_mesh.indices[3 * t];
        edge_uses[edge_key(tri[0], tri[1])]++;
        edge_uses[edge_key(tri[1], tri[2])]++;
        edge_uses[edge_key(tri[2], tri[0])]++;
    }
    std::vector<bool> on_open_boundary(vertex_count, false);
    for (std::size_t t = 0; t < triangle_count; t++) {
        const unsigned int *tri = &raw_mesh.indices[3 * t];
        for (int k = 0; k < 3; k++) {
            if (edge_uses[edge_key(tri[k], tri[(k + 1) % 3])] == 1) {
                on_open_boundary[tri[k]] = true;
                on_open_boundary[tri[(k + 1) % 3]] = true;
            }
        }
    }
    return on_open_boundary;
}

// Takes out the pieces of the trimmed surface that are not joined to the rest and are smaller than min_area, which the
// open boundary of the raw mesh does not touch (a piece at the edge of the reconstructed region may be the end of a
// larger surface): the specks that sound reflected from the water column, a fish or a bubble leave, and the end of an
// extrapolated sheet.
void remove_islands(const Mesh &raw_mesh, double min_area, std::vector<bool> *supported_vertices, SupportTrimmingDiagnostics *diagnostics) {
    const std::size_t vertex_count = raw_mesh.vertices.size();
    const std::size_t triangle_count = raw_mesh.indices.size() / 3;
    const std::vector<bool> on_open_boundary = open_boundary_vertices(raw_mesh);

    std::vector<unsigned int> parent(vertex_count);
    std::iota(parent.begin(), parent.end(), 0u);
    const auto find = [&](unsigned int v) {
        while (parent[v] != v) {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        return v;
    };
    std::vector<bool> kept(triangle_count, false);
    for (std::size_t t = 0; t < triangle_count; t++) {
        const unsigned int *tri = &raw_mesh.indices[3 * t];
        if (tri[0] >= vertex_count || tri[1] >= vertex_count || tri[2] >= vertex_count) {
            continue;
        }
        if ((*supported_vertices)[tri[0]] && (*supported_vertices)[tri[1]] && (*supported_vertices)[tri[2]]) {
            kept[t] = true;
            parent[find(tri[1])] = find(tri[0]);
            parent[find(tri[2])] = find(tri[0]);
        }
    }
    struct Island {
        double area = 0.0;
        bool open = false;
    };
    std::unordered_map<unsigned int, Island> islands;
    for (std::size_t t = 0; t < triangle_count; t++) {
        if (!kept[t]) {
            continue;
        }
        const unsigned int *tri = &raw_mesh.indices[3 * t];
        Island &island = islands[find(tri[0])];
        island.area += 0.5 * cross(raw_mesh.vertices[tri[1]] - raw_mesh.vertices[tri[0]], raw_mesh.vertices[tri[2]] - raw_mesh.vertices[tri[0]]).length();
        for (int k = 0; k < 3; k++) {
            island.open = island.open || on_open_boundary[tri[k]];
        }
    }
    std::size_t removed = 0;
    for (const auto &entry : islands) {
        if (!entry.second.open && entry.second.area < min_area) {
            removed++;
        }
    }
    if (removed == 0) {
        return;
    }
    for (std::size_t t = 0; t < triangle_count; t++) {
        if (!kept[t]) {
            continue;
        }
        const unsigned int *tri = &raw_mesh.indices[3 * t];
        const Island &island = islands[find(tri[0])];
        if (!island.open && island.area < min_area) {
            for (int k = 0; k < 3; k++) {
                (*supported_vertices)[tri[k]] = false;
            }
        }
    }
    if (diagnostics != nullptr) {
        diagnostics->islands_removed = removed;
    }
}

// Puts back the holes that trimming leaves inside the surface: a patch of rejected triangles (they share rejected
// vertices) that the open boundary of the raw mesh does not touch, so that it is surrounded by triangles that are kept,
// and no larger than max_area. The vertices of a hole that were rejected only because the surface is a few cells off
// the samples there, and the sparse patches of a rough bottom, are such holes.
void restore_holes(const Mesh &raw_mesh, double max_area, const std::function<bool(std::size_t)> &vertex_ok, std::vector<bool> *supported_vertices,
                   SupportTrimmingDiagnostics *diagnostics) {
    const std::size_t vertex_count = raw_mesh.vertices.size();
    const std::size_t triangle_count = raw_mesh.indices.size() / 3;

    const std::vector<bool> on_open_boundary = open_boundary_vertices(raw_mesh);

    // Patches: the rejected vertices of each rejected triangle are joined.
    std::vector<unsigned int> parent(vertex_count);
    std::iota(parent.begin(), parent.end(), 0u);
    const auto find = [&](unsigned int v) {
        while (parent[v] != v) {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        return v;
    };
    constexpr unsigned int none = std::numeric_limits<unsigned int>::max();
    std::vector<std::size_t> rejected_triangles;
    std::vector<unsigned int> anchor;   // a rejected vertex of each rejected triangle
    for (std::size_t t = 0; t < triangle_count; t++) {
        const unsigned int *tri = &raw_mesh.indices[3 * t];
        if (tri[0] >= vertex_count || tri[1] >= vertex_count || tri[2] >= vertex_count) {
            continue;
        }
        unsigned int first = none;
        for (int k = 0; k < 3; k++) {
            if ((*supported_vertices)[tri[k]]) {
                continue;
            }
            if (first == none) {
                first = tri[k];
            } else {
                parent[find(tri[k])] = find(first);
            }
        }
        if (first != none) {
            rejected_triangles.push_back(t);
            anchor.push_back(first);
        }
    }

    struct Patch {
        double area = 0.0;
        bool open = false;
        std::vector<std::size_t> triangles;
    };
    std::unordered_map<unsigned int, Patch> patches;
    for (std::size_t n = 0; n < rejected_triangles.size(); n++) {
        const std::size_t t = rejected_triangles[n];
        const unsigned int *tri = &raw_mesh.indices[3 * t];
        Patch &patch = patches[find(anchor[n])];
        patch.area += 0.5 * cross(raw_mesh.vertices[tri[1]] - raw_mesh.vertices[tri[0]], raw_mesh.vertices[tri[2]] - raw_mesh.vertices[tri[0]]).length();
        patch.triangles.push_back(t);
        for (int k = 0; k < 3; k++) {
            patch.open = patch.open || on_open_boundary[tri[k]];
        }
    }
    for (const auto &entry : patches) {
        const Patch &patch = entry.second;
        if (patch.open || patch.area > max_area) {
            continue;
        }
        bool acceptable = true;
        for (const std::size_t t : patch.triangles) {
            for (int k = 0; k < 3 && acceptable; k++) {
                const unsigned int v = raw_mesh.indices[3 * t + k];
                acceptable = (*supported_vertices)[v] || vertex_ok(v);
            }
            if (!acceptable) {
                break;
            }
        }
        if (!acceptable) {
            continue;
        }
        for (const std::size_t t : patch.triangles) {
            for (int k = 0; k < 3; k++) {
                (*supported_vertices)[raw_mesh.indices[3 * t + k]] = true;
            }
        }
        if (diagnostics != nullptr) {
            diagnostics->holes_restored++;
        }
    }
}

Mesh compact_supported_triangles(const Mesh &raw_mesh, const std::vector<bool> &supported_vertices) {
    Mesh trimmed_mesh;
    trimmed_mesh.indices.reserve(raw_mesh.indices.size());

    const unsigned int unmapped = std::numeric_limits<unsigned int>::max();
    std::vector<unsigned int> vertex_map(raw_mesh.vertices.size(), unmapped);

    for (std::size_t i = 0; i + 2 < raw_mesh.indices.size(); i += 3) {
        const unsigned int old_indices[3] = {
            raw_mesh.indices[i],
            raw_mesh.indices[i + 1],
            raw_mesh.indices[i + 2],
        };

        bool valid_triangle = true;
        for (const unsigned int old_index : old_indices) {
            if (old_index >= raw_mesh.vertices.size() || !supported_vertices[old_index]) {
                valid_triangle = false;
                break;
            }
        }
        if (!valid_triangle ||
            old_indices[0] == old_indices[1] ||
            old_indices[1] == old_indices[2] ||
            old_indices[2] == old_indices[0]) {
            continue;
        }

        const Vec3 face_normal = cross(raw_mesh.vertices[old_indices[1]] - raw_mesh.vertices[old_indices[0]],
                                       raw_mesh.vertices[old_indices[2]] - raw_mesh.vertices[old_indices[0]]);
        if (face_normal.length_squared() <= vec3_epsilon * vec3_epsilon) {
            continue;
        }

        for (const unsigned int old_index : old_indices) {
            if (vertex_map[old_index] == unmapped) {
                vertex_map[old_index] = static_cast<unsigned int>(trimmed_mesh.vertices.size());
                trimmed_mesh.vertices.push_back(raw_mesh.vertices[old_index]);
            }
            trimmed_mesh.indices.push_back(vertex_map[old_index]);
        }
    }

    trimmed_mesh.normals = calculate_vertex_normals(trimmed_mesh);
    return trimmed_mesh;
}

} // namespace

Mesh support_trimming(const Mesh &raw_mesh, const OrientedPointCloud &oriented_points, SupportTrimmingOptions options, SupportTrimmingDiagnostics *diagnostics) {
    if (diagnostics != nullptr) {
        *diagnostics = SupportTrimmingDiagnostics();
        diagnostics->input_vertices = raw_mesh.vertices.size();
        diagnostics->input_triangles = raw_mesh.indices.size() / 3;
    }

    if (!options.enabled) {
        if (diagnostics != nullptr) {
            diagnostics->supported_vertices = raw_mesh.vertices.size();
            diagnostics->output_vertices = raw_mesh.vertices.size();
            diagnostics->output_triangles = raw_mesh.indices.size() / 3;
        }
        return raw_mesh;
    }
    if (raw_mesh.vertices.empty() || raw_mesh.indices.empty() || oriented_points.empty()) {
        return Mesh();
    }

    const std::vector<Vec3> points = sample_positions(oriented_points);
    const KDTree tree(points);

    const std::size_t minimum_neighbors = static_cast<std::size_t>(std::max(1, options.minimum_neighbors));
    const bool needs_automatic_support_radius = !std::isfinite(options.support_radius) || options.support_radius <= 0.0;
    const bool needs_automatic_normal_offset = !std::isfinite(options.max_normal_offset) || options.max_normal_offset <= 0.0;

    double point_spacing = 0.0;
    if (needs_automatic_support_radius || needs_automatic_normal_offset) {
        point_spacing = estimate_point_spacing(points, tree, minimum_neighbors);
    }
    if (diagnostics != nullptr) {
        diagnostics->estimated_point_spacing = point_spacing;
    }

    const double support_radius = needs_automatic_support_radius ? automatic_support_radius_scale * point_spacing : options.support_radius;
    const double max_normal_offset = needs_automatic_normal_offset ? (point_spacing > vec3_epsilon ? automatic_normal_offset_scale * point_spacing : 0.5 * support_radius) : options.max_normal_offset;

    if (!std::isfinite(support_radius) || !std::isfinite(max_normal_offset) || support_radius <= vec3_epsilon || max_normal_offset <= 0.0) {
        return Mesh();
    }
    if (diagnostics != nullptr) {
        diagnostics->resolved_support_radius = support_radius;
        diagnostics->resolved_max_normal_offset = max_normal_offset;
    }

    const double minimum_normal_alignment = std::clamp(options.minimum_normal_alignment, 0.0, 1.0);
    const std::vector<Vec3> mesh_normals = raw_mesh.normals.size() == raw_mesh.vertices.size() ? raw_mesh.normals : calculate_vertex_normals(raw_mesh);

    std::vector<bool> supported_vertices(raw_mesh.vertices.size(), false);
    std::vector<VertexSupportResult> results(raw_mesh.vertices.size(), VertexSupportResult::insufficient_neighbors);
    for (std::size_t i = 0; i < raw_mesh.vertices.size(); i++) {
        const VertexSupportResult result =
            vertex_support_result(raw_mesh.vertices[i],
                                  mesh_normals[i],
                                  oriented_points,
                                  tree,
                                  support_radius,
                                  max_normal_offset,
                                  minimum_neighbors,
                                  minimum_normal_alignment);
        results[i] = result;
        supported_vertices[i] = result == VertexSupportResult::supported;

        if (diagnostics == nullptr) {
            continue;
        }

        switch (result) {
        case VertexSupportResult::supported:
            diagnostics->supported_vertices++;
            break;
        case VertexSupportResult::insufficient_neighbors:
            diagnostics->rejected_for_neighbors++;
            break;
        case VertexSupportResult::excessive_normal_offset:
            diagnostics->rejected_for_normal_offset++;
            break;
        case VertexSupportResult::insufficient_normal_alignment:
            diagnostics->rejected_for_normal_alignment++;
            break;
        }
    }

    // A vertex of a void, or of a hole, has soundings all around it, in the surface. Three tests tell it from the
    // extrapolation of a surface beyond the edge of its soundings, which has none on the far side, and from a sheet of
    // the mesh that hangs under the surveyed surface (the closure of the screened Poisson surface under an overhang),
    // whose soundings are all above it. The footprint, which is seen from above, and the distance to the nearest
    // sounding, cannot tell them apart:
    //  - the nearest sounding is no farther than max_void_distance (if that is set);
    //  - for a vertex farther than the support radius from its nearest sounding, the soundings near it are beside it
    //    and not above or below it;
    //  - the tangent plane of the mesh at the vertex is probed in eight directions, at twice the distance of the
    //    nearest sounding, and a sounding must be near each probe, from the raster of the footprint, which knows the
    //    soundings of the whole survey and their heights. Up to two of the eight may fail, for a vertex at the corner of
    //    a void or in the lanes between lines of soundings. Without a footprint the probes cannot fail.
    const auto surrounded_by_samples = [&](std::size_t i, double *distance) {
        const std::vector<KDTree::Neighbor> nearest = tree.k_nearest(raw_mesh.vertices[i], void_neighbor_count);
        if (nearest.empty()) {
            return false;
        }
        *distance = std::sqrt(nearest[0].distance_squared);
        if (options.max_void_distance > 0.0 && *distance > options.max_void_distance) {
            return false;
        }
        if (*distance > support_radius) {
            std::vector<double> alignments;
            for (const KDTree::Neighbor &neighbor : nearest) {
                const Vec3 to_sample = points[neighbor.index] - raw_mesh.vertices[i];
                const double length = to_sample.length();
                alignments.push_back(length > vec3_epsilon ? std::fabs(dot(to_sample, mesh_normals[i])) / length : 0.0);
            }
            std::nth_element(alignments.begin(), alignments.begin() + alignments.size() / 2, alignments.end());
            if (alignments[alignments.size() / 2] > void_maximum_normal_alignment) {
                return false;
            }
        }
        const Vec3 normal = normalize(mesh_normals[i]);
        const Vec3 axis = std::fabs(normal.z) < 0.9 ? Vec3(0.0, 0.0, 1.0) : Vec3(1.0, 0.0, 0.0);
        const Vec3 u = normalize(cross(normal, axis));
        const Vec3 w = cross(normal, u);
        const double reach = std::max(2.0 * *distance, 2.0 * support_radius);
        const double tolerance = std::max(void_probe_tolerance * *distance, 2.0 * support_radius);
        int uncovered = 0;
        for (int k = 0; k < 8; k++) {
            const double angle = k * 0.25 * 3.14159265358979323846;
            const Vec3 probe = raw_mesh.vertices[i] + (u * std::cos(angle) + w * std::sin(angle)) * reach;
            // The footprint knows the soundings of the whole survey, with their heights, so it is right at the border of
            // the chunk too, where the points of this chunk do not reach.
            if (options.footprint != nullptr && !options.footprint->has_sounding_near(probe.x, probe.y, probe.z, tolerance)) {
                uncovered++;
            }
        }
        return uncovered <= 2;
    };

    // A vertex that is only short of neighbors, and has a sounding close by, is not a void: the soundings are sparse there.
    // This puts back the pinholes and the sparse lanes between lines that the support radius, which is chosen from the
    // median spacing of the densest parts of the survey, does not reach.
    if (options.max_void_distance > 0.0) {
        for (std::size_t i = 0; i < raw_mesh.vertices.size(); i++) {
            if (supported_vertices[i] || results[i] != VertexSupportResult::insufficient_neighbors) {
                continue;
            }
            const std::vector<KDTree::Neighbor> nearest = tree.k_nearest(raw_mesh.vertices[i], 1);
            if (!nearest.empty() && std::sqrt(nearest[0].distance_squared) <= near_sounding_scale * support_radius) {
                supported_vertices[i] = true;
                if (diagnostics != nullptr) {
                    diagnostics->near_vertices_restored++;
                }
            }
        }
    }

    if (options.footprint != nullptr) {
        std::vector<double> distances;
        for (std::size_t i = 0; i < raw_mesh.vertices.size(); i++) {
            double distance = 0.0;
            if (supported_vertices[i] || results[i] != VertexSupportResult::insufficient_neighbors || !surrounded_by_samples(i, &distance)) {
                continue;
            }
            supported_vertices[i] = true;
            distances.push_back(distance);
        }
        if (diagnostics != nullptr && !distances.empty()) {
            diagnostics->void_vertices_restored = distances.size();
            std::nth_element(distances.begin(), distances.begin() + distances.size() / 2, distances.end());
            diagnostics->void_distance_median = distances[distances.size() / 2];
            diagnostics->void_distance_maximum = *std::max_element(distances.begin(), distances.end());
        }
    }

    if (options.max_hole_area > 0.0 && std::isfinite(options.max_hole_area)) {
        restore_holes(raw_mesh, options.max_hole_area, [&](std::size_t v) {
            double distance = 0.0;
            return surrounded_by_samples(v, &distance);
        }, &supported_vertices, diagnostics);
    }

    if (options.min_island_area > 0.0 && std::isfinite(options.min_island_area)) {
        remove_islands(raw_mesh, options.min_island_area, &supported_vertices, diagnostics);
    }

    Mesh trimmed_mesh = compact_supported_triangles(raw_mesh, supported_vertices);
    if (diagnostics != nullptr) {
        diagnostics->output_vertices = trimmed_mesh.vertices.size();
        diagnostics->output_triangles = trimmed_mesh.indices.size() / 3;
    }
    return trimmed_mesh;
}
