#
# Arrow configuration.
#

find_package(Arrow REQUIRED CONFIG)
if (Arrow_VERSION VERSION_LESS "21.0")
    message("Required at least Arrow version 21.0, but found ${Arrow_VERSION}")
endif()

find_package(Parquet REQUIRED CONFIG)
if (Parquet_VERSION VERSION_LESS "21.0")
    message("Required at least Parquet version 21.0, but found ${Parquet_VERSION}")
endif()
