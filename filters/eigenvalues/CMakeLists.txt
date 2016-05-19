PDAL_ADD_DRIVER(filter eigenvalues "EigenvaluesFilter.cpp" "EigenvaluesFilter.hpp" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
