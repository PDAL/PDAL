#
# Arrow configuration.
#

find_package(Arrow QUIET)
set_package_properties(Arrow PROPERTIES TYPE OPTIONAL)
if (Arrow_FOUND)
    set(PDAL_HAVE_ARROW 1)
    if (Arrow_VERSION VERSION_LESS "21.0")
        message(FATAL_ERROR "Required at least Arrow version 21.0, but found ${Arrow_VERSION}")
    endif()
    
    # Only look for Parquet if we found arrow already.
    find_package(Parquet QUIET)
    set_package_properties(Parquet PROPERTIES TYPE OPTIONAL)
    if (Parquet_FOUND)
        set(PDAL_HAVE_PARQUET 1)
        if (Parquet_VERSION VERSION_LESS "21.0")
            message(FATAL_ERROR "Required at least Parquet version 21.0, but found ${Parquet_VERSION}")
        endif()
    endif()
endif()
