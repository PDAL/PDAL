#------------------------------------------------------------------------------
# Custom command to generate Dimension.hpp from Dimension.json
#------------------------------------------------------------------------------

set(DIMENSION_INFILE ${PDAL_SRC_DIR}/Dimension.json)
set(DIMENSION_OUTFILE ${CMAKE_CURRENT_BINARY_DIR}/include/pdal/Dimension.hpp)

# In cross compiling scenarios, the location of the native dimbuilder executable
# can be provided to the CMake invocation (in cases not suitable for CMAKE_CROSSCOMPILING_EMULATOR).
set (DIMBUILDER_EXECUTABLE dimbuilder CACHE STRING "native dimbuilder executable")
IF (CMAKE_CROSSCOMPILING)
    SET(DIMBUILDER "${DIMBUILDER_EXECUTABLE}")
ELSE (CMAKE_CROSSCOMPILING)
    SET(DIMBUILDER dimbuilder)
ENDIF (CMAKE_CROSSCOMPILING)

add_custom_command(OUTPUT ${DIMENSION_OUTFILE}
    COMMAND ${DIMBUILDER_EXECUTABLE} ${DIMENSION_INFILE} ${DIMENSION_OUTFILE}
        DEPENDS ${DIMENSION_INFILE} ${DIMBUILDER})
add_custom_target(generate_dimension_hpp DEPENDS ${DIMENSION_OUTFILE})
