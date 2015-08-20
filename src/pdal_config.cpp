/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS version related functions.
 * Author:   Mateusz Loskot, mateusz@loskot.net
 *           Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 2008, Mateusz Loskot
 * Copyright (c) 2010, Frank Warmerdam
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following
 * conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 ****************************************************************************/

#include <pdal/pdal_config.hpp>

#include <sstream>
#include <iomanip>

#include <pdal/pdal_defines.h>
#include <pdal/gitsha.h>

#ifdef PDAL_HAVE_LIBGEOTIFF
#include <geotiff.h>
#endif

#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wfloat-equal"
#endif
#include <gdal.h>
#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic pop
#endif

#ifdef PDAL_HAVE_LASZIP
#include <laszip/laszip.hpp>
#endif

#ifdef PDAL_HAVE_GEOS
#include <geos/version.h>
#endif

#ifdef PDAL_HAVE_HEXER
#include <hexer/hexer.hpp>
#endif

#ifdef PDAL_HAVE_LIBXML2
#include <libxml/xmlversion.h>
#endif

#include <pdal/util/Utils.hpp>

namespace pdal
{

/// Check if GeoTIFF support has been built in to PDAL
bool IsLibGeoTIFFEnabled()
{
#ifdef PDAL_HAVE_LIBGEOTIFF
    return true;
#else
    return false;
#endif
}

/// Check if LasZip compression support has been built in to PDAL
bool IsLasZipEnabled()
{
#ifdef PDAL_HAVE_LASZIP
    return true;
#else
    return false;
#endif
}

int GetVersionMajor()
{
    return PDAL_VERSION_MAJOR;
}

int GetVersionMinor()
{
    return PDAL_VERSION_MINOR;
}

int GetVersionPatch()
{
    return PDAL_VERSION_PATCH;
}

std::string GetVersionString()
{
    return std::string(PDAL_VERSION_STRING);
}

int GetVersionInteger()
{
    return PDAL_VERSION_INTEGER;
}

std::string GetSHA1()
{
	return g_GIT_SHA1;
}


/// Tell the user a bit about PDAL's compilation
std::string GetFullVersionString()
{
    std::ostringstream os;

    std::string sha = GetSHA1();
    if (!Utils::iequals(sha, "Release"))
        sha = sha.substr(0,6);

    os << PDAL_VERSION_STRING << " (git-version: " << sha << ")";

    return os.str();
}


std::string getPDALDebugInformation()
{
    Utils::screenWidth();
    std::string headline(Utils::screenWidth(), '-');

    std::ostringstream os;

    os << headline << std::endl;
    os << "PDAL debug information" << std::endl ;
    os << headline << std::endl << std::endl;

    os << "Version information" << std::endl;
    os << headline << std::endl;
    os << "(" << pdal::GetFullVersionString() << ")" << std::endl;
    os << std::endl;

    os << "Debug build status" << std::endl;
    os << headline << std::endl;
    os << PDAL_BUILD_TYPE << std::endl << std::endl;

    os << "Enabled libraries" << std::endl;
    os << headline << std::endl << std::endl;

#ifdef PDAL_HAVE_GEOS
    os << "GEOS (" << GEOS_VERSION << ") - " <<
        "http://trac.osgeo.org/geos" << std::endl;
#endif

    os << "GDAL (" << GDALVersionInfo("RELEASE_NAME") << ") - " <<
        "http://www.gdal.org" << std::endl;

#ifdef PDAL_HAVE_HEXER
    os << "Hexer (" << HEXER_VERSION_MAJOR << '.' << HEXER_VERSION_MINOR <<
        '.' << HEXER_VERSION_PATCH << ") - " <<
        "http://github.com/hobu/hexer" << std::endl;
#endif

#ifdef PDAL_HAVE_LASZIP
    os << "LASzip (" << LASZIP_VERSION_MAJOR << "." << LASZIP_VERSION_MINOR <<
        "." << LASZIP_VERSION_REVISION << ") - " <<
        "http://laszip.org" << std::endl;
#endif

#ifdef PDAL_HAVE_LIBXML2
    os << "libxml (" << LIBXML_DOTTED_VERSION << ") - " <<
              "http://www.xmlsoft.org/" << std::endl;
#endif

#ifdef PDAL_HAVE_LIBGEOTIFF
    os << "libgeotiff (" << LIBGEOTIFF_VERSION << ") - " <<
        "http://trac.osgeo.org/geotiff" << std::endl;
#endif

#ifdef PDAL_HAVE_MRSID
    os << "MrSID - " << "http://www.lizardtech.com" << std::endl;
#endif

#ifdef PDAL_HAVE_NITRO
    os << "Nitro - " << "http://github.com/hobu/nitro" << std::endl;
#endif

#ifdef PDAL_HAVE_ORACLE
    os << "Oracle - " << "http://www.oracle.com" << std::endl;
#endif

#ifdef PDAL_HAVE_P2G
    os << "Points2grid = " <<
        "http://github.com/CRREL/points2grid" << std::endl;
#endif

#ifdef PDAL_HAVE_PYTHON
    os << "Python - " << "http://www.python.org" << std::endl;
#endif

#ifdef PDAL_HAVE_SQLITE
    os << "SQLite - " << "http://www.sqlite.org" << std::endl;
#endif

#ifdef PDAL_HAVE_POSTGRESQL
    os << "PostgreSQL - " <<
        "http://github.com/pramsey/pointcloud" << std::endl;
#endif

    return os.str();
}

} // namespace pdal
