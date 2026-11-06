#------------------------------------------------------------------------------
#    The MB-system:  CMakeLists.txt   3/6/2026
#
#    Copyright (c) 2026 by
#    [Your Team/Institution]
#
#    See README.md file for copying and redistribution conditions.
#------------------------------------------------------------------------------

message("In src/mbmesh")

set(MBMESH_SOURCES
  mbmesh.cpp
  settings.cpp
  src/io/coordinate_frame.cpp
  src/io/datalist_reader.cpp
  src/io/point_spill.cpp
  src/io/xyz_writer.cpp
  src/io/track.cpp
  src/io/provenance.cpp
  src/io/glb_writer.cpp
  src/io/draco_mesh.cpp
  src/io/tileset_writer.cpp
  src/io/x3dom_writer.cpp
  src/algorithms/point_decimation.cpp
  src/algorithms/normal_estimation.cpp
  src/algorithms/screened_poisson.cpp
  src/algorithms/marching_cubes.cpp
  src/algorithms/support_trimming.cpp
  src/algorithms/seam_blending.cpp
  src/algorithms/reconstruction.cpp)

find_package(Threads REQUIRED)

# Draco, for the compression of the triangle meshes of the GLB files (KHR_draco_mesh_compression): the encoder source that
# mbgrd2gltf holds, built here as a library of its own. Without it the GLBs are not compressed.
set(MBMESH_DRACO_DIR "${CMAKE_SOURCE_DIR}/src/mbgrd2gltf/draco")
if(EXISTS "${MBMESH_DRACO_DIR}/compression/encode.cc")
  file(GLOB_RECURSE MBMESH_DRACO_SOURCES "${MBMESH_DRACO_DIR}/*.cc")
  list(FILTER MBMESH_DRACO_SOURCES EXCLUDE REGEX ".*/.*test.*")
  add_library(mbmesh_draco STATIC ${MBMESH_DRACO_SOURCES})
  target_compile_features(mbmesh_draco PRIVATE cxx_std_17)
  target_include_directories(mbmesh_draco PUBLIC ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)
  target_compile_options(mbmesh_draco PRIVATE -w)
  set_target_properties(mbmesh_draco PROPERTIES POSITION_INDEPENDENT_CODE ON)
  set_source_files_properties(src/io/draco_mesh.cpp PROPERTIES COMPILE_DEFINITIONS MBMESH_WITH_DRACO)
  set(MBMESH_DRACO_LIBRARY mbmesh_draco)
else()
  message(WARNING "mbmesh: the Draco source was not found in src/mbgrd2gltf/draco; GLBs will not be compressed")
  set(MBMESH_DRACO_LIBRARY "")
endif()

add_executable(mbmesh ${MBMESH_SOURCES})
target_compile_features(mbmesh PRIVATE cxx_std_17)

# Link to MB-System mbio library
target_link_libraries(mbmesh PRIVATE mbio Threads::Threads ${MBMESH_DRACO_LIBRARY})

target_include_directories(mbmesh PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/src/mbio
  ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)

install(TARGETS mbmesh DESTINATION ${CMAKE_INSTALL_BINDIR})

if(buildTests)
  add_executable(mbmesh_test_support_trimming
    tests/test_support_trimming.cpp
    src/algorithms/support_trimming.cpp
    src/algorithms/screened_poisson.cpp
    src/algorithms/marching_cubes.cpp
    src/io/provenance.cpp
    src/io/glb_writer.cpp
    src/io/draco_mesh.cpp)
  target_compile_features(mbmesh_test_support_trimming PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_support_trimming PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)
  add_test(NAME mbmesh_support_trimming COMMAND mbmesh_test_support_trimming)

  add_executable(mbmesh_test_settings
    tests/test_settings.cpp
    settings.cpp
    src/io/coordinate_frame.cpp
    src/io/datalist_reader.cpp
    src/io/point_spill.cpp
    src/algorithms/point_decimation.cpp
    src/algorithms/normal_estimation.cpp
    src/algorithms/screened_poisson.cpp
    src/algorithms/marching_cubes.cpp
    src/algorithms/support_trimming.cpp
    src/algorithms/seam_blending.cpp
    src/algorithms/reconstruction.cpp)
  target_compile_features(mbmesh_test_settings PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_settings PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/mbio
    ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)
  target_link_libraries(mbmesh_test_settings PRIVATE mbio Threads::Threads)
  add_test(NAME mbmesh_settings COMMAND mbmesh_test_settings)

  add_executable(mbmesh_test_coverage_footprint
    tests/test_coverage_footprint.cpp
    src/algorithms/support_trimming.cpp)
  target_compile_features(mbmesh_test_coverage_footprint PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_coverage_footprint PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME mbmesh_coverage_footprint COMMAND mbmesh_test_coverage_footprint)

  add_executable(mbmesh_test_seam_blending
    tests/test_seam_blending.cpp
    src/algorithms/seam_blending.cpp
    src/algorithms/marching_cubes.cpp)
  target_compile_features(mbmesh_test_seam_blending PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_seam_blending PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME mbmesh_seam_blending COMMAND mbmesh_test_seam_blending)

  add_executable(mbmesh_test_point_spill
    tests/test_point_spill.cpp
    src/io/point_spill.cpp)
  target_compile_features(mbmesh_test_point_spill PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_point_spill PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME mbmesh_point_spill COMMAND mbmesh_test_point_spill)

  add_executable(mbmesh_test_coordinate_frame
    tests/test_coordinate_frame.cpp
    src/io/coordinate_frame.cpp)
  target_compile_features(mbmesh_test_coordinate_frame PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_coordinate_frame PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME mbmesh_coordinate_frame COMMAND mbmesh_test_coordinate_frame)

  add_executable(mbmesh_test_chunking
    tests/test_chunking.cpp
    src/io/point_spill.cpp
    src/algorithms/reconstruction.cpp
    src/algorithms/seam_blending.cpp
    src/algorithms/normal_estimation.cpp
    src/algorithms/screened_poisson.cpp
    src/algorithms/marching_cubes.cpp
    src/algorithms/support_trimming.cpp)
  target_compile_features(mbmesh_test_chunking PRIVATE cxx_std_17)
  target_link_libraries(mbmesh_test_chunking PRIVATE Threads::Threads)
  target_include_directories(mbmesh_test_chunking PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME mbmesh_chunking COMMAND mbmesh_test_chunking)

  add_executable(mbmesh_test_track
    tests/test_track.cpp
    src/io/track.cpp
    src/io/provenance.cpp
    src/io/glb_writer.cpp
    src/io/draco_mesh.cpp)
  target_compile_features(mbmesh_test_track PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_track PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)
  add_test(NAME mbmesh_track COMMAND mbmesh_test_track)

  add_executable(mbmesh_test_x3dom_writer
    tests/test_x3dom_writer.cpp
    src/io/coordinate_frame.cpp
    src/io/provenance.cpp
    src/io/x3dom_writer.cpp)
  target_compile_features(mbmesh_test_x3dom_writer PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_x3dom_writer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME mbmesh_x3dom_writer COMMAND mbmesh_test_x3dom_writer)

  add_executable(mbmesh_test_tileset
    tests/test_tileset.cpp
    src/io/coordinate_frame.cpp
    src/io/provenance.cpp
    src/io/tileset_writer.cpp)
  target_compile_features(mbmesh_test_tileset PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_tileset PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)
  add_test(NAME mbmesh_tileset COMMAND mbmesh_test_tileset)

  add_executable(mbmesh_test_provenance
    tests/test_provenance.cpp
    src/io/coordinate_frame.cpp
    src/io/provenance.cpp
    src/io/glb_writer.cpp
    src/io/draco_mesh.cpp
    src/io/x3dom_writer.cpp)
  target_compile_features(mbmesh_test_provenance PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_provenance PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)
  add_test(NAME mbmesh_provenance COMMAND mbmesh_test_provenance)

  add_executable(mbmesh_test_marching_cubes
    tests/test_marching_cubes.cpp
    src/algorithms/marching_cubes.cpp
    src/io/provenance.cpp
    src/io/glb_writer.cpp
    src/io/draco_mesh.cpp)
  target_compile_features(mbmesh_test_marching_cubes PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_marching_cubes PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)
  add_test(NAME mbmesh_marching_cubes COMMAND mbmesh_test_marching_cubes)

  add_executable(mbmesh_test_normal_estimation
    tests/test_normal_estimation.cpp
    src/algorithms/normal_estimation.cpp
    src/io/provenance.cpp
    src/io/glb_writer.cpp
    src/io/draco_mesh.cpp)
  target_compile_features(mbmesh_test_normal_estimation PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_normal_estimation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)
  add_test(NAME mbmesh_normal_estimation COMMAND mbmesh_test_normal_estimation)

  add_executable(mbmesh_test_draco_mesh
    tests/test_draco_mesh.cpp
    src/io/provenance.cpp
    src/io/glb_writer.cpp
    src/io/draco_mesh.cpp)
  target_compile_features(mbmesh_test_draco_mesh PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_draco_mesh PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)
  add_test(NAME mbmesh_draco_mesh COMMAND mbmesh_test_draco_mesh)

  # draco_mesh.cpp is built with Draco in it, so every test that holds it needs the library.
  if(MBMESH_DRACO_LIBRARY)
    get_property(mbmesh_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
    foreach(mbmesh_target ${mbmesh_targets})
      get_target_property(mbmesh_target_sources ${mbmesh_target} SOURCES)
      if(mbmesh_target_sources MATCHES "draco_mesh.cpp" AND NOT mbmesh_target STREQUAL "mbmesh")
        target_link_libraries(${mbmesh_target} PRIVATE ${MBMESH_DRACO_LIBRARY})
      endif()
    endforeach()
  endif()

  add_test(NAME mbmesh_test COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/tests/mbmesh_test.py")
endif()
