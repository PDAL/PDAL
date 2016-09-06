PDAL_ADD_DRIVER(filter iqr "IQRFilter.cpp" "IQRFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
