/*
 * pdal_defines.h.in is used by cmake to generate hexer_defines.h
 *
 * Do not edit hexer_defines.h!
 *
 * Note this is a C-style header.  Do not use C++ syntax.
 */
#ifndef HEXER_DEFINES_H
#define HEXER_DEFINES_H

/*
 * version settings
 */
#define HEXER_VERSION_MAJOR 1
#define HEXER_VERSION_MINOR 4
#define HEXER_VERSION_PATCH 0

#define HEXER_VERSION_STRING "1.4.0"


/*
 * availability of 3rd-party libraries
 */
#define HEXER_HAVE_CAIRO
#define HEXER_HAVE_GDAL

/*
 * availability of execinfo and backtrace
 */
#define HEXER_HAVE_EXECINFO_H

/*
 * Debug or Release build?
 */
#define HEXER_BUILD_TYPE "Release"

/*
 * platform compiler
 */
/* #undef HEXER_COMPILER_MSVC */

#endif
