#
# SPZ library
#

# The oldest spz release that's on conda-forge is 2.0.0,
# but they have it marked as 1.1 in their cmake file.
find_package(spz 1.1.0 REQUIRED)
set_package_properties(spz PROPERTIES TYPE REQUIRED
    PURPOSE "SPZ gaussian splat format support")
if(spz_FOUND)
    set(PDAL_HAVE_SPZ 1)
endif(spz_FOUND)
