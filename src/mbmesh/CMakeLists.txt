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
  src/io/xyz_writer.cpp
  src/io/track.cpp
  src/io/glb_writer.cpp
  src/io/x3dom_writer.cpp
  src/algorithms/point_decimation.cpp
  src/algorithms/normal_estimation.cpp
  src/algorithms/screened_poisson.cpp
  src/algorithms/marching_cubes.cpp
  src/algorithms/support_trimming.cpp
  src/algorithms/reconstruction.cpp)

find_package(Threads REQUIRED)

add_executable(mbmesh ${MBMESH_SOURCES})
target_compile_features(mbmesh PRIVATE cxx_std_17)

# Link to MB-System mbio library
target_link_libraries(mbmesh PRIVATE mbio Threads::Threads)

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
    src/io/glb_writer.cpp)
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
    src/algorithms/point_decimation.cpp
    src/algorithms/normal_estimation.cpp
    src/algorithms/screened_poisson.cpp
    src/algorithms/marching_cubes.cpp
    src/algorithms/support_trimming.cpp
    src/algorithms/reconstruction.cpp)
  target_compile_features(mbmesh_test_settings PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_settings PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/mbio
    ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)
  target_link_libraries(mbmesh_test_settings PRIVATE mbio Threads::Threads)
  add_test(NAME mbmesh_settings COMMAND mbmesh_test_settings)

  add_executable(mbmesh_test_coordinate_frame
    tests/test_coordinate_frame.cpp
    src/io/coordinate_frame.cpp)
  target_compile_features(mbmesh_test_coordinate_frame PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_coordinate_frame PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME mbmesh_coordinate_frame COMMAND mbmesh_test_coordinate_frame)

  add_executable(mbmesh_test_chunking
    tests/test_chunking.cpp
    src/algorithms/reconstruction.cpp
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
    src/io/glb_writer.cpp)
  target_compile_features(mbmesh_test_track PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_track PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)
  add_test(NAME mbmesh_track COMMAND mbmesh_test_track)

  add_executable(mbmesh_test_marching_cubes
    tests/test_marching_cubes.cpp
    src/algorithms/marching_cubes.cpp
    src/io/glb_writer.cpp)
  target_compile_features(mbmesh_test_marching_cubes PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_marching_cubes PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)
  add_test(NAME mbmesh_marching_cubes COMMAND mbmesh_test_marching_cubes)

  add_executable(mbmesh_test_normal_estimation
    tests/test_normal_estimation.cpp
    src/algorithms/normal_estimation.cpp
    src/io/glb_writer.cpp)
  target_compile_features(mbmesh_test_normal_estimation PRIVATE cxx_std_17)
  target_include_directories(mbmesh_test_normal_estimation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/mbgrd2gltf)
  add_test(NAME mbmesh_normal_estimation COMMAND mbmesh_test_normal_estimation)

  add_test(NAME mbmesh_test COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/tests/mbmesh_test.py")
endif()
