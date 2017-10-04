#
# Options for optional components.
#

option(WITH_COMPLETION
    "Install bash completions scripts for command line?" FALSE)
add_feature_info("Bash completion" WITH_COMPLETION
    "completion for PDAL command line")

option(BUILD_PLUGIN_CPD
    "Choose if the cpd filter should be built" FALSE)
add_feature_info("CPD plugin" BUILD_PLUGIN_CPD
    "Coherent Point Drift (CPD) computes rigid or nonrigid transformations between point sets")

option(BUILD_PLUGIN_GEOWAVE
    "Choose if GeoWave support should be built" FALSE)
add_feature_info("GeoWave plugin" BUILD_PLUGIN_GEOWAVE
    "Read and Write data using GeoWave")

option(BUILD_PLUGIN_GREYHOUND
    "Choose if Greyhound support should be built" FALSE)
add_feature_info("Greyhound plugin" BUILD_PLUGIN_GREYHOUND
    "read points from a Greyhound server")

option(BUILD_PLUGIN_HEXBIN
    "Choose if the HexBin filter is built" FALSE)
add_feature_info("Hexbin plugin" BUILD_PLUGIN_HEXBIN
    "determine boundary and density of a point cloud")

option(BUILD_PLUGIN_ICEBRIDGE
    "Choose if Icebridge support should be built" FALSE)
add_feature_info("Icebridge plugin" BUILD_PLUGIN_ICEBRIDGE
    "read data in the Icebridge format")

option(BUILD_PLUGIN_MATLAB
    "Choose if Matlab support should be built" FALSE)
add_feature_info("Matlab plugin" BUILD_PLUGIN_MATLAB
    "write data to a .mat file")

option(BUILD_PLUGIN_MRSID
    "Choose if MrSID/LiDAR support should be built" FALSE)
add_feature_info("MrSID plugin" BUILD_PLUGIN_MRSID
    "read data in the MrSID format")

option(BUILD_PLUGIN_NITF
    "Choose if NITF support should be built (only install supported is from http://github.com/hobu/nitro)" FALSE)
add_feature_info("NITF plugin" BUILD_PLUGIN_NITF
    "read/write LAS data wrapped in NITF")

option(BUILD_PLUGIN_OPENSCENEGRAPH
    "Choose if OpenSceneGraph support should be built" FALSE)
add_feature_info("OpenSceneGraph plugin" BUILD_PLUGIN_OPENSCENEGRAPH
    "read/write OpenSceneGraph objects")

if(DEFINED ENV{ORACLE_HOME})
    set(DEFINED_ORACLE_HOME TRUE)
else(DEFINED ENV{ORACLE_HOME})
    set(DEFINED_ORACLE_HOME FALSE)
endif(DEFINED ENV{ORACLE_HOME})
option(BUILD_PLUGIN_OCI
    "Choose if OCI support should be built" ${DEFINED_ORACLE_HOME})

option(BUILD_PLUGIN_PCL "Choose if PCL support should be built" FALSE)
add_feature_info("PCL plugin" BUILD_PLUGIN_PCL
    "provides PCL-based readers, writers, filters, and kernels")

find_package(PostgreSQL QUIET)
option(BUILD_PLUGIN_PGPOINTCLOUD
    "Choose if PostgreSQL PointCloud support should be built"
    ${POSTGRESQL_FOUND})
add_feature_info("PostgreSQL PointCloud plugin" BUILD_PLUGIN_PGPOINTCLOUD
    "read/write PostgreSQL PointCloud objects")

option(BUILD_PLUGIN_SQLITE
    "Choose if SQLite database support should be built" FALSE)
add_feature_info("SQLite plugin" BUILD_PLUGIN_SQLITE
    "read/write SQLite objects")

option(BUILD_PLUGIN_RIVLIB
    "Choose if RiVLib support should be built" FALSE)
add_feature_info("RiVLib plugin" BUILD_PLUGIN_RIVLIB
    "read data in the RXP format")

option(BUILD_PLUGIN_PYTHON
    "Choose if Python support should be built" FALSE)
add_feature_info("Python plugin" BUILD_PLUGIN_PYTHON
    "add features that depend on python")

option(BUILD_PLUGIN_MBIO
    "Choose if MBIO support should be built" FALSE)
add_feature_info("MBIO plugin" BUILD_PLUGIN_MBIO
    "add features that depend on MBIO")

option(BUILD_TOOLS_NITFWRAP "Choose if nitfwrap tool should be built" FALSE)

option(WITH_TESTS
    "Choose if PDAL unit tests should be built" TRUE)
add_feature_info("Unit tests" WITH_TESTS "PDAL unit tests")

# Enable CTest and submissions to PDAL dashboard at CDash
# http://my.cdash.org/index.php?project=PDAL
option(ENABLE_CTEST
    "Enable CTest to support submissions of results to CDash at http://cdash.org" FALSE)

#
# Choose dependent options
#

include(CMakeDependentOption)

cmake_dependent_option(BUILD_PGPOINTCLOUD_TESTS
    "Choose if PostgreSQL PointCloud tests should be built"
    ON "BUILD_PLUGIN_PGPOINTCLOUD; WITH_TESTS" OFF)
cmake_dependent_option(BUILD_SQLITE_TESTS
    "Choose if SQLite tests should be built"
    ON "BUILD_PLUGIN_SQLITE; WITH_TESTS" OFF)
cmake_dependent_option(BUILD_OCI_TESTS
    "Choose if OCI tests should be built"
    ON "BUILD_PLUGIN_OCI; WITH_TESTS" OFF)
cmake_dependent_option(BUILD_RIVLIB_TESTS
    "Choose if RiVLib tests should be built"
    ON "BUILD_PLUGIN_RIVLIB; WITH_TESTS" OFF)
cmake_dependent_option(BUILD_PIPELINE_TESTS
    "Choose if pipeline tests should be built"
    OFF "WITH_TESTS" OFF)
