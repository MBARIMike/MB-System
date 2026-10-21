# mbmesh

## Overview

**mbmesh** reads swath sonar data through MB-System's datalist processing framework and reconstructs a triangle mesh from accepted multibeam echosounder (MBES) bathymetry soundings.

The current implementation is an end-to-end mesh-generation prototype. It reads MB-System swath files, converts valid soundings and sensor origins into a local Cartesian frame, estimates dataset spacing, derives reconstruction parameters from either a requested or automatic level of detail, estimates oriented normals, reconstructs a scalar field with screened Poisson, extracts a raw mesh with marching cubes, trims unsupported regions, and writes a binary glTF mesh.

`mesh.glb` vertices are WGS84 Earth-Centered, Earth-Fixed (ECEF) coordinates in meters, using the same conventions as `mbgrd2gltf`:

- **No `--geoorigin`** (default): absolute ECEF coordinates.
- **`--geoorigin` / `-G` with no value**: ECEF offsets from the GeoOrigin at the center of the data bounds and the mean sounding elevation.
- **`--geoorigin=LON,LAT,ELEV` / `-GLON,LAT,ELEV`**: ECEF offsets from the specified GeoOrigin.

Offsets are a pure translation (`vertex = ECEF(point) - ECEF(GeoOrigin)`); the axes stay aligned with ECEF, exactly as in `mbgrd2gltf --geoorigin`. A `mbmesh` and an `mbgrd2gltf` product made with the same GeoOrigin therefore line up directly. GLB positions are 32-bit floats, so absolute ECEF (magnitudes of about 6.4 million meters) resolves only about 0.5 m; use a GeoOrigin to preserve sub-meter detail. `mbmesh` prints a warning when no GeoOrigin is given.

Internally, reconstruction runs in a separate local east-north-up (ENU) frame tangent to the WGS84 ellipsoid at the point below the data centroid. Each sounding and sensor position is converted to ECEF and then rigidly translated and rotated into this frame, so there is no map-projection distortion, and results are converted back to ECEF exactly. This frame is an implementation detail and does not depend on `--geoorigin`. `pointcloud-local.xyz` is written in it.

The default output is:

- **`mesh.glb`** - support-trimmed final triangle mesh in binary glTF format, using the `mbmesh` local Cartesian coordinate frame

Optional command-line flags can also write:

- **`raw_mesh.glb`** - untrimmed marching-cubes mesh before support trimming
- **`pointcloud-local.xyz`** - accepted/decimated samples in the same local reconstruction frame as `mesh.glb`
- **`pointcloud-ecef.xyz`** - accepted/decimated samples in WGS84 Earth-Centered, Earth-Fixed coordinates
- **`oriented-pointcloud-ecef.ply`** - oriented samples and PCA lambda values
- **`pointcloud.glb`** - diagnostic point-cloud GLB
- **`normals.glb`** - diagnostic normal-vector GLB
- **`origins.glb`** - diagnostic sensor-origin ray GLB
- **`mesh.html`** - X3DOM viewer page for `mesh.glb`

## Quick Start

Build `mbmesh` from the MB-System repository root:

```bash
cmake --build build --target mbmesh
```

Run a basic reconstruction:

```bash
./build/src/mbmesh/mbmesh -I datalist.mb-1
```

This writes:

```text
mbmesh_output/mesh.glb
```

Run with a custom output directory, requested level of detail, raw mesh output, and verbose logging:

```bash
./build/src/mbmesh/mbmesh -I datalist.mb-1 -L 0.5 -O ./mbmesh_output --raw-mesh-glb -V
```

Inspect metadata before running the full reconstruction:

```bash
./build/src/mbmesh/mbmesh -I datalist.mb-1 --info
```

Generate an HTML viewer:

```bash
./build/src/mbmesh/mbmesh -I datalist.mb-1 -O ./mbmesh_output --html
```

When `--html` is used, `mbmesh` writes `mesh.html` and prints, as a log message, the command to view it. It does not start a server or a browser, so no process is left running and no port is held. Run the printed command in a terminal (Ctrl-C stops it) and open the printed URL:

```text
mbmesh: [html] to view mesh.html, run this command in a terminal (Ctrl-C stops it), then open the URL:
mbmesh: [html]   python3 -m http.server 8000 --bind 127.0.0.1 --directory '/path/to/mbmesh_output'
mbmesh: [html]   http://127.0.0.1:8000/mesh.html
```

The page has to be served over HTTP because browsers will not load `mesh.glb` from a `file://` URL.

## Command-Line Interface

| Option | Argument | Description |
| --- | --- | --- |
| `-I`, `--input` | `datalist` | Input MB-System datalist path. Required unless the datalist is supplied as the first non-option argument. |
| `-O`, `--output` | `outputdir` | Output directory. Defaults to `mbmesh_output`. |
| `-R`, `--bounds` | `west/east/south/north` | Geographic subset bounds in degrees. Use `--bounds=-122.5/-121.8/36.5/37.2` when the west value is negative. |
| `-G`, `--geoorigin` | `[lon,lat,elev]` | Write GLB vertices as ECEF offsets from a GeoOrigin, as in `mbgrd2gltf`. With a value, all three of lon, lat, elev (degrees, degrees, meters above the ellipsoid) are required. With no value, uses the data-bounds center and mean elevation. Without the option, vertices are absolute ECEF. The value must be attached, as in `mbgrd2gltf`: `--geoorigin=-122.4,36.6,0` or `-G-122.4,36.6,0`. |
| `-L`, `--lod`, `--level-of-detail` | `meters` | Requested smallest feature size in local meters; the Poisson cell is a quarter of it, but no finer than half the point spacing. If omitted, `mbmesh` derives LOD from the measured point spacing, coarsened if needed to keep the run within about 500 million cells. |
| `--chunk-size` | `meters` | Reconstruct in square tiles of about this size (rounded up to whole Poisson cells) instead of one pass. Chunking turns on automatically when a single Poisson grid would exceed about 8 million cells. |
| `--chunk` | `i,j` | Reconstruct only tile (i,j), for fast testing. Tile (i,j) covers x in [i*size, (i+1)*size) and y in [j*size, (j+1)*size) in the local east-north frame; use `--info` and a verbose run to see tile indices. |
| `--max-chunks` | `n` | Stop after n tiles (for testing). |
| `--threads` | `n` | Tiles reconstructed at once when chunking. Defaults to the smaller of the core count and 4. Each tile holds its own Poisson grid, so memory grows with this. |
| `--decimate`, `-decimate` | `meters` | Enable voxel-grid point decimation with the specified cell size. |
| `--info` | none | Read the datalist, print dataset information (counts, bounds, point spacing) and the reconstruction plan (cell size, tile plan, cell count), and exit before mesh reconstruction. This option was called `--metadata` in earlier versions; `--metadata` is no longer accepted. |
| `--html`, `-html` | none | Write `mesh.html`, an X3DOM viewer page, and print the command to serve and view it. |
| `--xyz`, `-xyz` | none | Write both `pointcloud-local.xyz` and `pointcloud-ecef.xyz`. |
| `--local-xyz`, `-local-xyz` | none | Write `pointcloud-local.xyz`. |
| `--ecef-xyz`, `-ecef-xyz` | none | Write `pointcloud-ecef.xyz`. |
| `--oriented-ply`, `-oriented-ply` | none | Write `oriented-pointcloud-ecef.ply`. |
| `--pointcloud-glb`, `-pointcloud-glb` | none | Write `pointcloud.glb`. |
| `--normal-glb`, `-normal-glb` | none | Write `normals.glb`. |
| `--origin-glb`, `-origin-glb` | none | Write `origins.glb`. |
| `--raw-mesh-glb`, `-raw-mesh-glb` | none | Write `raw_mesh.glb`, the untrimmed mesh before support trimming. |
| `--diagnostics`, `-diagnostics` | none | Write `pointcloud.glb`, `normals.glb`, and `origins.glb`. |
| `--all-outputs`, `-all-outputs` | none | Write every optional output currently supported. |
| `-V`, `--verbose` | none | Enable progress and diagnostic logging. |
| `-H`, `-h`, `--help` | none | Print command usage and exit. |

Notes:

- The final `mesh.glb` is always written for successful runs without `--info`.
- `--xyz` overlaps with `--local-xyz` and `--ecef-xyz`.
- `--diagnostics` overlaps with `--pointcloud-glb`, `--normal-glb`, and `--origin-glb`.
- Point decimation is disabled by default and enabled only with `--decimate <meters>`.

## Output Files

### Default Output

1. **`mesh.glb`** - Final support-trimmed triangle mesh
   - Always written on successful reconstruction runs
   - Uses binary glTF format
   - Vertices are ECEF, or ECEF offsets from the GeoOrigin with `--geoorigin`, matching `mbgrd2gltf`
   - Intended for web and desktop GLB viewers

### Chunked Output

When reconstruction is chunked (see "Chunked Reconstruction"), `mesh.glb` is still written as the merged mesh, and each tile is also written as `tiles/tile_I_J.glb`, with vertices placed exactly as in `mesh.glb`.

### Optional Outputs

2. **`raw_mesh.glb`** - Raw marching-cubes mesh
   - Written with `--raw-mesh-glb` or `--all-outputs`
   - Useful for comparing the mesh before and after support trimming

3. **`mesh.html`** - X3DOM viewer page
   - Written with `--html` or `--all-outputs`
   - References `mesh.glb`; with `--geoorigin` it places the model with `GeoOrigin` and `GeoViewpoint` nodes as `mbgrd2gltf` does
   - The command to serve it locally is printed when it is generated; mbmesh does not start a server or browser

4. **`pointcloud-local.xyz`** - Local Cartesian point cloud
   - Written with `--local-xyz`, `--xyz`, or `--all-outputs`
   - Uses the same local reconstruction frame as `mesh.glb`

5. **`pointcloud-ecef.xyz`** - ECEF point cloud
   - Written with `--ecef-xyz`, `--xyz`, or `--all-outputs`
   - Uses WGS84 Earth-Centered, Earth-Fixed coordinates

6. **`oriented-pointcloud-ecef.ply`** - Oriented point cloud
   - Written with `--oriented-ply` or `--all-outputs`
   - Includes ECEF position, ECEF normal, and PCA lambda values

7. **`pointcloud.glb`** - Diagnostic point-cloud GLB
   - Written with `--pointcloud-glb`, `--diagnostics`, or `--all-outputs`

8. **`normals.glb`** - Diagnostic normal-vector GLB
   - Written with `--normal-glb`, `--diagnostics`, or `--all-outputs`

9. **`origins.glb`** - Diagnostic sensor-origin ray GLB
   - Written with `--origin-glb`, `--diagnostics`, or `--all-outputs`

## Pipeline Summary

The current pipeline is orchestrated by `src/mbmesh/mbmesh.cpp`:

1. Parse CLI options.
2. Read and preprocess the MB-System datalist.
3. Print metadata and exit if requested.
4. Optionally decimate collected points.
5. Reconstruct the surface, either in one pass or tile by tile (steps 5a-5d are the same either way, and run once per tile when chunked):
   - 5a. Estimate oriented normals.
   - 5b. Reconstruct a screened-Poisson scalar field.
   - 5c. Extract a raw mesh with marching cubes.
   - 5d. Trim unsupported mesh regions.
6. Write `mesh.glb`, per-tile GLBs when chunked, and requested optional outputs.
7. Print the command to view `mesh.html` if `--html` was requested.

## Pipeline Details

### 1. Datalist Reading

Implemented in:

- `include/io/datalist_reader.h`
- `src/io/datalist_reader.cpp`

An MB-System datalist contains one or more swath sonar files. Each swath file contains pings, and each ping contains many beams. A valid bathymetry beam produces a sounding: a measured seafloor point.

`read_datalist()` uses MB-System's datalist and MBIO APIs to:

- Open the datalist with `mb_datalist_open()`
- Select processed files when the datalist marks them for use
- Read swath records with `mb_get_all()`
- Process only `MB_DATA_DATA` records
- Count files, pings, soundings, accepted soundings, flagged soundings, and out-of-bounds soundings
- Reject flagged beams with `mb_beam_ok()` when configured to do so
- Apply optional geographic bounds from `-R` / `--bounds`
- Convert accepted soundings into local Cartesian coordinates
- Store the sonar sensor origin for each accepted sounding

Soundings and sensor positions are converted to WGS84 ECEF while reading and held in absolute ECEF until all files have been read. The reconstruction frame origin is then chosen (the point on the ellipsoid below the centroid, rounded to 1e-6 degrees) and every point is converted in place to the local east-north-up frame described in the Overview. Sounding elevation is stored as negative bathymetry and sensor elevation as negative sensor depth, both treated as heights above the WGS84 ellipsoid. The conversions live in `include/io/coordinate_frame.h` and `src/io/coordinate_frame.cpp`.

On output, mesh and diagnostic GLB vertices are converted from the local frame back to ECEF and offset by the `--geoorigin` position when one is used (normals rotate but do not translate).

Each accepted sample is stored as a `CollectedPoint`:

- `point` - local Cartesian sounding position
- `origin` - local Cartesian sonar sensor position for the ping

The sensor origin is later used to orient PCA normals.

### 2. Metadata and Dataset Analysis

Implemented in:

- `settings.cpp`
- `settings.h`

`preprocess_datalist()` wraps `read_datalist()` and computes metadata needed for reconstruction defaults:

- Accepted point count
- Local 3D bounds
- Geodetic longitude/latitude bounds
- Nearest-neighbor spacing statistics. Consecutive soundings are spatially adjacent (consecutive beams of consecutive pings), so spacing is measured inside blocks of consecutive points, 16 blocks of 16,384 points spread through the dataset (the whole dataset if it has fewer than 262,144 points). This is the spacing along a survey line, and it does not depend on how many points there are. An earlier version searched a subsample of 8,192 points spread over the whole dataset, which overstated the spacing by about the square root of the number of points divided by 8,192, so dense surveys looked sparse and were reconstructed far too coarsely.

Use metadata mode to inspect a dataset before reconstruction:

```bash
./build/src/mbmesh/mbmesh -I datalist.mb-1 --info
```

### 3. Level of Detail and Automatic Defaults

Implemented in:

- `settings.cpp`
- `settings.h`

The `-L`, `--lod`, or `--level-of-detail` option is the main user-facing reconstruction control. It represents the requested smallest feature size in local meters. If it is omitted, `mbmesh` derives an automatic LOD from the estimated average point spacing.

The Poisson cell size is a quarter of the level of detail, but no finer than half the point spacing, since the data cannot support more: `cell = max(L / 4, spacing / 2)`. It is not capped at the spacing, so `-L` is honored even when the points are much denser than the level of detail asked for. To match a 5 cm grid, ask for `-L 0.2`.

The automatic level of detail is the finest the data supports, twice the point spacing, unless that would need more than about 500 million Poisson cells across the survey's bounding box, in which case it is raised to fit and an `INFO` message says so. Work grows as the cube of the resolution, so a dense survey would otherwise ask for a centimeter-scale cell and run for days. A level of detail given with `-L` is never limited, but a `WARN` message is printed when it needs more than about 2 billion cells, since that may take hours. `--info` shows the resulting cell size, tile plan and cell count before you commit to a run.

`settings.cpp` clamps the resolved level of detail to the supported range and derives:

- Decimation cell size
- Normal-estimation search radius
- Normal-estimation neighbor count
- Poisson grid cell size
- Poisson padding
- Normal splat radius
- Poisson screening weight
- Poisson solver iteration count
- Marching-cubes iso-value
- Support-trimming radius
- Support-trimming normal-offset threshold

Warnings are printed when:

- The resolved LOD is not well supported by the estimated average point spacing.
- The normal-estimation radius is large relative to the requested feature size.

### 4. Point Decimation

Implemented in:

- `include/algorithms/point_decimation.h`
- `src/algorithms/point_decimation.cpp`

Point decimation reduces a `CollectedPointCloud` by voxel-grid averaging. Points are grouped into uniform 3D cells, and each occupied cell emits one averaged point and one averaged sensor origin.

Current behavior:

- The implementation exists and is part of the pipeline.
- The current spacing-driven defaults set `options.decimation.decimate = false`.
- The `--decimate <meters>` option enables decimation and sets the voxel cell size.

This stage is useful for reducing large datasets before normal estimation and reconstruction. Smaller cell sizes preserve more detail; larger values reduce downstream cost more aggressively.

### 5. PCA Normal Estimation

Implemented in:

- `include/algorithms/normal_estimation.h`
- `src/algorithms/normal_estimation.cpp`

`normal_estimation()` converts `CollectedPoint` samples into `OrientedPoint` samples:

- A KD-tree is built over the local point positions.
- The algorithm prefers a physical-radius neighbor search when `search_radius > 0`.
- Radius searches are capped by `k` to avoid unbounded neighborhoods in dense data.
- If the radius search finds fewer than `minimum_neighbors`, the algorithm falls back to k-nearest neighbors.
- PCA is performed on the query point plus its neighbors.
- The eigenvector associated with the smallest eigenvalue is used as the local normal direction.
- The normal is flipped, if needed, to point toward the sonar sensor origin.
- Eigenvalues are retained in `OrientedPoint::lambdas`.

The sensor-origin orientation step is important. PCA produces an unoriented axis: both `n` and `-n` fit the same local plane. Orienting toward the ping's sensor origin is more appropriate for complex bathymetry than a blanket `+Z` assumption.

Known limitation: PCA normals can become noisy near sharp transitions, sparse regions, inconsistent point spacing, or survey edges. Normal quality strongly affects the screened-Poisson reconstruction and support trimming stages.

### 6. Screened Poisson Reconstruction

Implemented in:

- `include/algorithms/screened_poisson.h`
- `src/algorithms/screened_poisson.cpp`

`screened_poisson()` converts oriented samples into a regular 3D scalar grid:

1. Build a padded reconstruction domain around the oriented samples.
2. Splat unit normals into nearby grid nodes with compact quadratic weights.
3. Build screening weights around observed samples.
4. Compute the divergence of the splatted normal field with finite differences.
5. Solve the screened Poisson equation with Jacobi relaxation.
6. Shift the scalar field so sample locations cluster around the configured iso-value.

The output is a `ScalarGrid3D` whose `iso_value` is normally `0.0`.

Screened Poisson was chosen because it can reconstruct surfaces with complex 3D structure, including overhang-like geometry that 2.5D gridding cannot represent. Its main tradeoff is that it tends to produce smooth, closed surfaces. Sparse regions and incorrect normals can create artifacts or closures that need downstream trimming.

The current implementation uses a regular grid, not an adaptive octree. Large high-resolution reconstructions can therefore be memory intensive. Chunked processing or an adaptive octree would be major future improvements.

### 7. Marching Cubes

Implemented in:

- `include/algorithms/marching_cubes.h`
- `src/algorithms/marching_cubes.cpp`

`marching_cubes()` extracts a triangle mesh from the scalar field. It classifies each grid cube against the configured iso-value and uses lookup tables to generate triangles for intersected cells.

The result is the raw reconstructed surface. Write it with:

```bash
./build/src/mbmesh/mbmesh -I datalist.mb-1 --raw-mesh-glb
```

The raw mesh is useful for debugging because it shows the screened-Poisson and marching-cubes output before support trimming removes unsupported regions.

Future candidate: dual contouring may represent sharp edges and abrupt orientation changes better than marching cubes.

### 8. Oriented Sample Support Trimming

Implemented in:

- `include/algorithms/support_trimming.h`
- `src/algorithms/support_trimming.cpp`

Screened Poisson and marching cubes can produce closed surfaces and unsupported artifacts. `support_trimming()` removes generated triangles that are not supported by nearby oriented input samples.

Current trimming logic:

- Build a KD-tree over oriented sample positions.
- Estimate point spacing if automatic radius or offset thresholds are needed.
- Compute mesh vertex normals if the raw mesh does not provide them.
- For each mesh vertex, gather nearby oriented samples within the support radius.
- Reject vertices with too few contributing neighbors.
- Reject vertices whose weighted point-to-plane offset is too large.
- Optionally reject vertices whose mesh normal is poorly aligned with nearby sample normals.
- Keep only triangles whose three vertices are supported.
- Compact surviving vertices and recompute vertex normals.

Current defaults:

- Support trimming is enabled.
- Minimum neighbors defaults to `2`.
- Minimum normal alignment defaults to `0.0`, which disables the alignment test.
- Support radius and normal offset are derived from point spacing and LOD-driven settings.

If trimming removes the entire mesh, `mbmesh` exits with an error instead of writing an empty final surface.

Known limitation: boundary triangles with one or two rejected vertices are removed entirely rather than clipped, which can leave ragged edges.

### 9. Output Generation

Implemented in:

- `include/io/glb_writer.h`
- `src/io/glb_writer.cpp`
- `include/io/xyz_writer.h`
- `src/io/xyz_writer.cpp`
- `include/io/x3dom_writer.h`
- `src/io/x3dom_writer.cpp`

`write_outputs()` always writes the final mesh as `mesh.glb`. Other output writers run only when their CLI flags are set.

Local XYZ output writes local reconstruction coordinates. ECEF XYZ and oriented PLY outputs convert local points back through the coordinate frame and then into WGS84 ECEF coordinates. The HTML writer creates a minimal X3DOM page titled `MB-System mbmesh Mesh Viewer`.

When `--html` is used, `print_html_viewer_command()` logs the command to serve the output directory (`python3 -m http.server 8000 --bind 127.0.0.1 --directory <absolute outputdir>`) and the URL to open (`http://127.0.0.1:8000/mesh.html`). It does not start the server.

## Chunked Reconstruction

Implemented in:

- `include/algorithms/reconstruction.h`
- `src/algorithms/reconstruction.cpp`

A single screened-Poisson grid covers the bounding box of its input. A seafloor survey is a thin sheet through a large 3D box, so the grid is mostly empty, and its memory and run time grow quickly with survey size and level of detail. Chunking splits the survey into square horizontal tiles and reconstructs each one separately.

How it works:

- **Tile lattice.** Tile (i, j) owns x in [i*T, (i+1)*T) and y in [j*T, (j+1)*T) in the local east-north frame, where T is the tile size, a whole number of Poisson cells. The lattice is anchored at the local-frame origin, so tile indices do not depend on the order of the data.
- **Shared grid lattice.** Every tile's Poisson grid is snapped to the same global lattice of cell-size steps, so neighboring tiles share grid nodes and their surfaces nearly coincide where they overlap.
- **Halo.** A tile reads its own points plus a ring of neighboring points, so that points near the tile edge see the neighborhoods a single run would give them. The halo covers the reach of normal splatting or support trimming plus the normal-estimation radius plus a few cells.
- **Ownership.** After trimming, a tile keeps only triangles whose centroid lies inside its half-open square, so each triangle belongs to exactly one tile and the merged mesh has no overlap.
- **Automatic sizing.** Without `--chunk-size`, mbmesh turns chunking on when the whole-survey grid would exceed about 8 million cells, and chooses the tile size so each tile's grid, including halo and padding, is about that size. A tile's grid is as tall as the relief inside it, so the vertical extent is measured from a sample of the points as the tallest relief in any one tile of that size, not the survey's full depth range, which would make tiles needlessly small on a sloping seafloor. The memory budget alone can give only a few huge tiles, which leaves most threads idle, so the tile is then made smaller until at least two tiles per thread contain points, down to a floor of four halos across (so that the halo and padding, which every tile reconstructs again, stay a modest cost). Asking for more threads therefore gives smaller tiles. An explicit `--chunk-size` is used as given. `--info` prints the resulting tile size, halo, grid size, number of tiles with points and thread count.
- **Threads.** Tiles are independent, so several are reconstructed at once (`--threads`). The tiles with the most points are started first, so the last tiles to start are the small ones and the threads finish together. Results are delivered in tile order, so the output does not depend on the thread count or the start order.

Measured on one 175 MB swath file (993,245 accepted soundings, about 50 m by 37 m) at `-L 2`, a single run took 207 s and 1.4 GB. Tiled with 10 m tiles, it took 254 s on one thread and 69 s on four threads at a peak of 0.76 GB, with 151,057 triangles against 151,148 for the single run. The median distance from a tiled vertex to the nearest single-run vertex was 4.9 cm, about the vertex spacing. At what was then the default level of detail (a 0.098 m cell from a spacing estimate that was too large, about 102 million cells for the whole survey, which a single run refuses to attempt) automatic chunking produced 20 tiles of about 8.3 m and 528,208 triangles in 232 s at a peak of 1.8 GB on four threads. Three survey-edge tiles with fewer than 32 owned points were skipped.

On the same file at `-L 2` with automatic tiling, 4 threads took 66 s for 8 tiles with points, 8 threads 45 s for 16 tiles, and 12 threads 48 s for 20 tiles, so beyond about 8 threads the gain stops.

Current limitations:

- All soundings are still read into memory before tiling; only the Poisson grids are chunked. A survey too large to hold in memory still needs a spill-to-disk binning pass.
- Each tile computes its own iso-value shift from its own samples, and the tiles are not welded, so there can be small steps or cracks at tile edges. The halo keeps these small; they have been checked on a synthetic surface and one real file only.
- Per-tile run time is not reduced, only memory per run, so small surveys can run slower chunked on one thread than in one pass because the halo and padding are reconstructed more than once.
- Boundary triangles are assigned by centroid and are not clipped at tile edges.

## Legacy Implementation

The previous implementation in `src/mbmesh/legacy/` was created by the Spring 2026 CSUMB Capstone project team. It read MB-System datalists and generated point-cloud outputs:

- `adjustedPointcloud.xyz`
- `ecefPointcloud.xyz`
- `adjustedPointcloud.glb`
- `adjustedPointcloud.html`

The legacy tool used a single large `mbmesh.cpp` file plus a point-cloud GLB writer. It had `-html` support and could launch a local viewer, but mesh generation was future work in that version.

The current implementation keeps the same broad input domain and web-viewer direction, but replaces the point-cloud-first program with modular mesh reconstruction code:

- Algorithm stages live under `include/algorithms/` and `src/algorithms/`.
- Datalist, GLB, XYZ/PLY, and X3DOM output are separate I/O modules.
- Runtime defaults and metadata live in `settings.cpp`.
- Final output is `mesh.glb` by default, with point-cloud files now optional diagnostics.

## Current Status and Limitations

**mbmesh** is a functioning mesh-generation prototype. It is useful for reconstructing and inspecting bathymetry meshes, but several parts are still research/prototype quality.

Important limitations:

- Reconstruction is a single end-to-end run; there is no reusable preprocessing artifact yet.
- Decimation is available through `--decimate`, but remains off by default because it trades reconstruction detail for lower downstream cost.
- Screened Poisson uses a regular grid, so memory per run grows with grid size. Chunked reconstruction bounds this per tile, but all soundings are still held in memory.
- Normal estimation is sensitive to sparse data, sharp edges, and inconsistent spacing.
- Support trimming can over-trim when normals or thresholds are poor.
- Boundary triangles are discarded rather than clipped when only part of a triangle is unsupported.
- Viewing `mesh.html` needs `python3` (or any local web server) and a browser; mbmesh only prints the command.

High-value future work:

- Write a 3D Tiles `tileset.json` with an ENU-to-ECEF `transform`, so tiles do not depend on the glb vertex offsets.
- Expose selected reconstruction parameters as documented CLI options.
- Bin soundings to per-tile files on disk so surveys larger than memory can be chunked.
- Weld or clip tile edges, and make the iso-value shift consistent across tiles.
- Investigate octree/adaptive Poisson reconstruction.
- Improve normal estimation near boundaries and sharp orientation changes.
- Clip support-trimmed boundary triangles instead of dropping them.
- Add full CLI regression tests using small real or synthetic datalists.
- Improve the generated HTML viewer with metadata and controls.
- Evaluate 3D Tiles and compression outputs for large web visualization workflows.

## Source Layout

### Main Application

- **mbmesh.cpp** - CLI parsing, pipeline orchestration, output selection, help text, and the viewer command message
- **settings.cpp** - Runtime defaults, metadata computation, spacing estimation, bounds handling, and metadata output
- **settings.h** - CLI option state and metadata structures

### Algorithm Interfaces

- **include/algorithms/point_decimation.h** - Point decimation options and interface
- **include/algorithms/normal_estimation.h** - Normal estimation options and interface
- **include/algorithms/screened_poisson.h** - Screened-Poisson options and scalar-grid interface
- **include/algorithms/marching_cubes.h** - Marching-cubes options and mesh extraction interface
- **include/algorithms/support_trimming.h** - Support-trimming options, diagnostics, and interface

### Algorithm Implementations

- **src/algorithms/point_decimation.cpp** - Voxel-grid point decimation
- **src/algorithms/normal_estimation.cpp** - PCA normal estimation with sensor-origin orientation
- **src/algorithms/screened_poisson.cpp** - Regular-grid screened-Poisson scalar-field reconstruction
- **src/algorithms/marching_cubes.cpp** - Triangle extraction from scalar grids
- **src/algorithms/support_trimming.cpp** - Support-based mesh filtering

### Data Structures and Math Helpers

- **include/data_types/geometry.h** - Point, mesh, bounds, reconstruction-domain, and grid data types
- **include/data_types/swathfile.h** - Swath-file data types
- **include/util/stopwatch.h** - Wall-clock timer used for the timing messages
- **include/util/logger.h** - Header-only logger producing the same line format as `mbgrd2gltf`
- **include/math/vec2.h**, **vec3.h**, **mat3.h** - Basic vector and matrix helpers
- **include/math/eigen.h** - Symmetric 3x3 eigen decomposition
- **include/math/kdtree.h** - KD-tree nearest-neighbor search

- **include/algorithms/reconstruction.h** and **src/algorithms/reconstruction.cpp** - Per-tile reconstruction pipeline and the chunked, threaded tile driver

### I/O

- **include/io/coordinate_frame.h** and **src/io/coordinate_frame.cpp** - Geodetic, ECEF, and local east-north-up conversions
- **include/io/datalist_reader.h** and **src/io/datalist_reader.cpp** - MB-System datalist and swath reading
- **include/io/glb_writer.h** and **src/io/glb_writer.cpp** - Mesh, point, normal, and origin-ray GLB writers
- **include/io/xyz_writer.h** and **src/io/xyz_writer.cpp** - Local XYZ, ECEF XYZ, and oriented PLY writers
- **include/io/x3dom_writer.h** and **src/io/x3dom_writer.cpp** - HTML/X3DOM viewer writer

### Tests

- **tests/test_marching_cubes.cpp** - Marching-cubes test program
- **tests/test_normal_estimation.cpp** - Normal-estimation test program
- **tests/test_support_trimming.cpp** - Support-trimming test program
- **tests/test_chunking.cpp** - Chunked reconstruction test (accuracy, coverage, ownership, thread-count independence) on a synthetic surface
- **tests/test_coordinate_frame.cpp** - Geodetic/ECEF/local frame conversion tests
- **tests/synthetic_data.h** and **tests/perlin_noise.h** - Synthetic test data helpers

### Build Configuration

- **CMakeLists.txt** - CMake build configuration for the `mbmesh` executable and tests
- **Makefile.am** - Autotools build configuration

## Building

### CMake

From the MB-System repository root:

```bash
cmake --build build --target mbmesh
```

If the build directory has not been configured yet:

```bash
cmake -S . -B build
cmake --build build --target mbmesh
```

The resulting executable is:

```bash
./build/src/mbmesh/mbmesh
```

### Autotools

From the MB-System repository root:

```bash
./configure
make mbmesh
```

## Log Messages

`mbmesh` logs in the same line format as `mbgrd2gltf`: the level, the local date and time to the millisecond, the source file, function and line of the call, then the message:

```text
INFO 2026-10-19 14:35:15.419 mbmesh.cpp main():151 [startup] input=datalist.mb-1 output=mbmesh_output
DEBUG 2026-10-19 14:35:15.886 reconstruction.cpp reconstruct_chunked():290 [chunking] tile_size=6.05 m, halo=0.72 m, 6 tiles contain points
```

A normal run logs how long reading the input and the whole reconstruction took, and the total (`[input] completed ... in 1.4 s`, `[reconstruction] completed in 1 min 7.5 s on 4 threads`, `[complete] ..., total 1 min 9.0 s`). With `-V` each stage of each tile also reports its own time, which shows where the time goes.

By default `INFO`, `WARN` and `ERROR` lines are shown. `-V` adds `DEBUG` lines, which carry the progress messages for each stage and, when chunking, each tile (the tile name follows the stage). Unlike `mbgrd2gltf`, every level is written to standard error, so standard output carries only data such as the `--info` report. Output is serialized, so lines from tiles reconstructed on different threads do not interleave. The logger is `include/util/logger.h`; it is a separate, header-only implementation of the `mbgrd2gltf` format rather than shared code.

## Development Notes

- If `mbmesh` on the shell resolves to `/usr/local/bin/mbmesh`, it may be an older installed binary. Use `./build/src/mbmesh/mbmesh` to run the current build-tree executable, or reinstall after building.
- The CMake target is `mbmesh`.
- Only the final mesh output is enabled by default. Optional diagnostic outputs must be requested explicitly.
- Metadata mode still reads the datalist and computes spacing, but it exits before normal estimation, screened Poisson, marching cubes, trimming, and output writing.

## Copyright and License

These source files are copyright by David W. Caress, Dale N. Chayes, and the California State University Monterey Bay Capstone team. They are licensed using GPL3 as part of MB-System.
