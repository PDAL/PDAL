#
# SpatiaLite cmake configuration
#

include(FindPkgConfig)

pkg_search_module(SPATIALITE spatialite>=4.2.0)
if(SPATIALITE_FOUND)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMOD_SPATIALITE")
endif(SPATIALITE_FOUND)
