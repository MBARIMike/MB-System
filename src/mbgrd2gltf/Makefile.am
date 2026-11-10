bin_PROGRAMS = mbgrd2gltf

AM_CFLAGS =
AM_CFLAGS += ${libnetcdf_CFLAGS}

AM_CPPFLAGS =
AM_CPPFLAGS += ${libnetcdf_CPPFLAGS}
AM_CPPFLAGS += -I$(top_srcdir)/src/mbdraco

# The Draco compression is shared with mbmesh and mbglb2tiles (src/mbdraco, the convenience library libmbdraco.la).
mbgrd2gltf_CXXFLAGS = -std=c++17
mbgrd2gltf_SOURCES = main.cpp bathymetry.cpp geometry.cpp logger.cpp model.cpp options.cpp provenance.cpp viewpoint.cpp
mbgrd2gltf_LDADD =
mbgrd2gltf_LDADD += $(top_builddir)/src/mbdraco/libmbdraco.la
mbgrd2gltf_LDADD += ${libnetcdf_LIBS}
