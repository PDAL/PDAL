# Find version components
string(REGEX REPLACE "^([0-9]+).*" "\\1"
    PDAL_VERSION_MAJOR "${PDAL_VERSION_STRING}")
string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1"
    PDAL_VERSION_MINOR "${PDAL_VERSION_STRING}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1"
    PDAL_VERSION_PATCH "${PDAL_VERSION_STRING}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.[0-9]+(.*)" "\\1"
    PDAL_CANDIDATE_VERSION "${PDAL_VERSION_STRING}")
