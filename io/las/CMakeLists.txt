#
# LAS driver CMake configuration
#

if ((GEOTIFF_FOUND) AND (GDAL_FOUND))
    set (PDAL_DRIVERS_LAS_GTIFF GeotiffSupport.cpp)
endif()

if (LASZIP_FOUND)
    set(PDAL_DRIVERS_LAS_LASZIP ZipPoint.cpp)
endif()

set (srcs
  ${PDAL_DRIVERS_LAS_GTIFF}
  ${PDAL_DRIVERS_LAS_LASZIP}
  LasHeader.cpp
  LasUtils.cpp
  SummaryData.cpp
  VariableLengthRecord.cpp
)

set(incs
  GeotiffSupport.hpp
  HeaderVal.hpp
  LasError.hpp
  LasHeader.hpp
  LasUtils.hpp
  SummaryData.hpp
  VariableLengthRecord.hpp
  ZipPoint.hpp
)

set(objs "")

add_library(lascommon OBJECT ${srcs} ${incs})
set(objs ${objs} $<TARGET_OBJECTS:lascommon>)

#
# LAS Reader
#
set(srcs
    LasReader.cpp
)

set(incs
    LasReader.hpp
)

PDAL_ADD_DRIVER(reader las "${srcs}" "${incs}" reader_objs)
set(objs ${objs} ${reader_objs})

#
# LAS Writer
#
set(srcs
    LasWriter.cpp
)

set(incs
    LasWriter.hpp
)

PDAL_ADD_DRIVER(writer las "${srcs}" "${incs}" writer_objs)
set(objs ${objs} ${writer_objs})

set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objs} PARENT_SCOPE)
