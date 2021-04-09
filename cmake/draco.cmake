#
option(WITH_DRACO "Choose if Draco support should be built" FALSE)
if (WITH_DRACO)
    # the `pkg_check_modules` function is created with this call
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(DRACO REQUIRED draco>1.4.0)

    set_package_properties(Draco PROPERTIES TYPE RECOMMENDED
        PURPOSE "Provides Draco compression")
    set(PDAL_HAVE_DRACO 1)
else()
    set(WITH_DRACO FALSE)
endif()
