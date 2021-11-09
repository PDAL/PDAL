#------------------------------------------------------------------------------
# Custom command to generate Dimension.hpp from Dimension.json
#------------------------------------------------------------------------------

set(DIMENSION_INFILE ${PDAL_SRC_DIR}/Dimension.json)
set(DIMENSION_OUTFILE ${CMAKE_CURRENT_BINARY_DIR}/include/pdal/Dimension.hpp)

# In cross compiling scenarios, the location of the native DIMBUILDER_EXECUTABLE
# must be provided to the CMake invocation.
IF (CMAKE_CROSSCOMPILING)
    SET(DIMBUILDER "${DIMBUILDER_EXECUTABLE}")
ELSE (CMAKE_CROSSCOMPILING)
    SET(DIMBUILDER dimbuilder)
ENDIF (CMAKE_CROSSCOMPILING)

add_custom_command(OUTPUT ${DIMENSION_OUTFILE}
    COMMAND ${DIMBUILDER} ${DIMENSION_INFILE} ${DIMENSION_OUTFILE}
        DEPENDS ${DIMENSION_INFILE} dimbuilder)
add_custom_target(generate_dimension_hpp DEPENDS ${DIMENSION_OUTFILE})
