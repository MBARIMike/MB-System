# mbglb2tiles

Makes an OGC 3D Tiles tileset of levels of detail from a GLB mesh written by `mbmesh` or `mbgrd2gltf`, without
reconstructing anything: the mesh is read, coarsened level by level, and cut into tiles.

```
mbglb2tiles -I mesh.glb [-O outputdir] [--tile-size meters] [--weld-tolerance meters] [--threads n]
            [--no-draco] [--draco-precision meters] [--draco-normal-bits n] [--html [--x3dom-url url] [--sse pixels]] [--lift[=meters]] [--no-colors] [-V]
```

The output directory (`tileset` beside the input by default, where `mbmesh --tileset` writes its tileset) gets `tileset.json` and one directory `L<level>` of tile GLBs
(`tile_I_J.glb`) for each level, `L0` being the finest, the layout of `mbmesh --tileset`. A tileset already in the directory, from either program, is replaced: its
`tileset.json`, `L<level>` directories and flat `tile_I_J.glb` files are removed first, and the log says so. The survey page of `mbmesh` finds a tileset in a directory
of any name.

With `--html` it also writes a viewer page, `<outputdir>.html` beside the output directory, that shows the tileset with an X3DOM
`GeoTileset` node (a GeoOrigin at the origin of the frame, an overview viewpoint from above, the processing log in `mbglb2tiles.log` in the
output directory), and prints the command to serve and view it. `--x3dom-url` names the X3DOM script, which must be a build that has `GeoTileset`;
`--sse` is the `maximumScreenSpaceError` of that node, 1 pixel by default (X3DOM's own 8 refines tiles of centimeter errors only within a few meters).

Tiles are colored by height unless `--no-colors`: every vertex has a `COLOR_0` (bytes, Draco-compressed with the rest) from the Haxby table over the heights of the whole mesh, so every tile and level agrees, with a white base color so the colors show unchanged. `tileset.json` records the bounds as `heightColors` in its extras, and the `--html` page has a "color by height" checkbox that turns the vertex colors off, which shows the tiles in gray.

`--lift` puts the tileset above the ellipsoid, in the transform of `tileset.json`, and works as `mbmesh --geoorigin` does: without it the tileset has the true heights (and a warning says so if any are below the ellipsoid); with `--lift` alone the lift is the default below; with `--lift=900` it is that many meters. `GeoTileset` takes the height of the camera as at least 1 m,
so a camera below the ellipsoid, as over the seafloor, never gets close enough to refine a tile and only the root tile loads. The default lift is enough to
put the lowest point of the mesh 10 m above the ellipsoid, to a meter, if it is lower; it is logged and written in the tileset's `extras` as `liftMeters`. The page's viewpoints have the lift added.

The log says what is happening and which options are in effect, default or not: a `[settings]` line for each, then a message at the start and end of
each stage (reading and decoding, the frame, welding, each level, the tree, the page).

## What it does

1. **Reads the mesh.** Every primitive of every mesh, Draco-compressed or not, is joined into one mesh. The GLB must carry the
   `georeference` block that `mbmesh` and `mbgrd2gltf` write in `asset.extras` (WGS 84 ECEF, absolute or relative to a center);
   any other GLB is refused, because where it is on the Earth is not known.
2. **Welds.** The primitives of a GLB each have their own copies of the vertices on their edges, each quantized on its own, so the
   vertices closer than `--weld-tolerance` (by default twice the Draco step the file records, or 1 mm) are made one, and the normals are made again
   from the whole surface, so the shading is smooth across the edges of the primitives and of the tiles.
3. **Puts it in a local frame.** The east-north-up frame at the middle of the bounds of the mesh. The tiles are in that frame, written Y-up as glTF
   requires, and the `transform` of the root tile of `tileset.json` is the frame in ECEF. A mesh in absolute ECEF float32 coordinates resolves only
   about 0.5 m (a warning says so); make it with `--geoorigin`.
4. **Makes the levels.** The finest level is the mesh. Each level above clusters the vertices of the mesh on a lattice of cubes whose side is
   the spacing of the mesh times two to the power of the level, each cluster becoming its mean, and drops the triangles that collapse or repeat.
   The whole mesh is coarsened at once and then cut, so the tiles of a level meet exactly along their edges.
5. **Cuts the levels into tiles.** The tiles of a level are the squares of a lattice (anchored at a corner of the bounds) of side
   `--tile-size` times two to the power of the level, by default 256 times the spacing at the finest level, as `mbgrd2gltf` writes
   blocks of 256 cells; each triangle goes to the tile its center is in. Four tiles of a level are replaced by one of the next, until one
   tile holds everything, so every tile has about as many vertices as any other.
6. **Writes `tileset.json`.** Version 1.1, `REPLACE` refinement, a box bounding volume for each tile that holds its content and its children, and
   a `geometricError` of the cell size of the level (0 for the finest tiles). A tile with no content of its own, where the coarsening left none, is
   written without `content`.

The tile GLBs are compressed with Draco (`--no-draco`, `--draco-precision` and `--draco-normal-bits` as in `mbmesh`, from `src/mbdraco`). A
tile is one primitive.

## Checked

`3d-tiles-validator` reports no errors or warnings, only that it cannot validate the Draco extension. A mesh of 38 million vertices
and 154 primitives (lidar of OceanImaging2018) gave 6 levels and 380 tiles, 255 MiB, in 13 minutes with a peak of 12 GB, before the Draco
primitives were decoded on several threads. Loading the tileset in X3DOM's `GeoTileset` and CesiumJS has not been tried.

## Limits

- The whole mesh is read into memory (about 100 bytes per vertex while it is being welded and clustered). A mesh of tens of millions of
  vertices takes gigabytes; building the levels from the chunk files of `mbmesh` instead is not done.
- The tiles are in one frame for the whole tileset; a survey tens of kilometers across would need a frame (a `transform`) for each tile to
  keep the float32 coordinates fine.
- The coarse levels are simplified by clustering vertices, not by edge collapse with an error measure, so thin features and sharp edges round off.
- Only the CMake build.
