#------------------------------------------------------------------------------
# Custom command to generate Dimension.hpp from Dimension.json
#------------------------------------------------------------------------------

set(DIMENSION_INFILE ${PDAL_SRC_DIR}/Dimension.json)
set(DIMENSION_OUTFILE ${CMAKE_CURRENT_BINARY_DIR}/include/pdal/Dimension.hpp)
add_custom_command(OUTPUT ${DIMENSION_OUTFILE}
    COMMAND dimbuilder ${DIMENSION_INFILE} ${DIMENSION_OUTFILE}
        DEPENDS ${DIMENSION_INFILE} dimbuilder)
add_custom_target(generate_dimension_hpp DEPENDS ${DIMENSION_OUTFILE})
