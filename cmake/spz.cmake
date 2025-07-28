#
# SPZ library
#

find_package(spz REQUIRED)
set_package_properties(spz PROPERTIES TYPE REQUIRED
    PURPOSE "SPZ gaussian splat format support")
if(spz_FOUND)
    set(PDAL_HAVE_SPZ 1)
endif(spz_FOUND)
