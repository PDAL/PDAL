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

    os << "PDAL " << PDAL_VERSION_STRING << " (" <<
        GetSHA1().substr(0, 6) << ")";

    return os.str();
}

std::string getPDALDebugInformation()
{
    std::string headline("------------------------------------------------------------------------------------------");

    std::ostringstream os;

    os << headline << std::endl;
    os << "PDAL debug information" << std::endl ;
    os << headline << std::endl << std::endl;

    os << "Version information" << std::endl;
    os << headline << std::endl;
    os << "(" << pdal::GetFullVersionString() << ")" << std::endl;
    os << std::endl;

    os << "Debug build status" << std::endl;
    os << headline << std::endl << std::endl;
    os << PDAL_BUILD_TYPE << std::endl<< std::endl;

    os << "Enabled libraries" << std::endl;
    os << headline << std::endl << std::endl;

    uint32_t special_column(32);
    uint32_t name_column(20);
    uint32_t url_column(40);

    std::ostringstream thdr;
    for (unsigned i = 0; i < name_column-1; ++i)
        thdr << "=";
    thdr << " ";
    for (unsigned i = 0; i < url_column-1; ++i)
        thdr << "=";
    thdr << " ";
    for (unsigned i = 0; i < special_column-1; ++i)
        thdr << "=";
    thdr << " ";

    name_column--;

    unsigned step_back(3);

    os << thdr.str() << std::endl;
    os << std::setw(name_column-step_back) << "Name" << std::setw(url_column-step_back) << "URL"  << std::setw(special_column-step_back) << "Version" << std::endl;
    os << thdr.str() << std::endl ;

#ifdef PDAL_HAVE_GEOS
    os << std::left
              << std::setw(name_column) << "GEOS" << std::right
              << std::setw(url_column) << "http://trac.osgeo.org/geos"
              << std::setw(special_column) << GEOS_VERSION  << std::endl;
#endif

    os << std::left
              << std::setw(name_column) << "GDAL" << std::right
              << std::setw(url_column) << "http://www.gdal.org"
              << std::setw(special_column) << GDALVersionInfo("RELEASE_NAME")  << std::endl;

#ifdef PDAL_HAVE_HEXER
    std::ostringstream hexerver;
    hexerver << HEXER_VERSION_MAJOR << "."
        << HEXER_VERSION_MINOR << "."
        << HEXER_VERSION_PATCH;
    os << std::left
              << std::setw(name_column) << "Hexer" << std::right
              << std::setw(url_column) << "http://github.com/hobu/hexer/"
              << std::setw(special_column) << hexerver.str()  << std::endl;
#endif
#ifdef PDAL_HAVE_LASZIP

    std::ostringstream laszipver;
    laszipver << LASZIP_VERSION_MAJOR << "."
        << LASZIP_VERSION_MINOR << "."
        << LASZIP_VERSION_REVISION;
    os << std::left
              << std::setw(name_column) << "LASzip" << std::right
              << std::setw(url_column) << "http://laszip.org"
              << std::setw(special_column) << laszipver.str() << std::endl;

#endif
#ifdef PDAL_HAVE_LIBXML2
    os << std::left
              << std::setw(name_column) << "libxml" << std::right
              << std::setw(url_column) << "http://www.xmlsoft.org/"
              << std::setw(special_column) << LIBXML_DOTTED_VERSION  << std::endl;
#endif
#ifdef PDAL_HAVE_LIBGEOTIFF
    os << std::left
              << std::setw(name_column) << "libgeotiff" << std::right
              << std::setw(url_column) << "http://trac.osgeo.org/geotiff/"
              << std::setw(special_column) << LIBGEOTIFF_VERSION  << std::endl;
#endif
#ifdef PDAL_HAVE_MRSID
    os << std::left
              << std::setw(name_column) << "MrSID" << std::right
              << std::setw(url_column) << "http://www.lizardtech.com"
              << std::setw(special_column) << ""  << std::endl;
#endif
#ifdef PDAL_HAVE_MSGPACK
    os << std::left
              << std::setw(name_column) << "MessagePack" << std::right
              << std::setw(url_column) << "http://msgpack.org/"
              << std::setw(special_column) << ""  << std::endl;
#endif
#ifdef PDAL_HAVE_NITRO
    os << std::left
              << std::setw(name_column) << "Nitro" << std::right
              << std::setw(url_column) << "http://github.com/hobu/nitro/"
              << std::setw(special_column) << ""  << std::endl;
#endif
#ifdef PDAL_HAVE_ORACLE
    os << std::left
              << std::setw(name_column) << "Oracle" << std::right
              << std::setw(url_column) << "http://www.oracle.com/"
              << std::setw(special_column) << ""  << std::endl;
#endif
#ifdef PDAL_HAVE_P2G
    os << std::left
              << std::setw(name_column) << "Points2grid" << std::right
              << std::setw(url_column) << "http://github.com/CRREL/points2grid/"
              << std::setw(special_column) << ""  << std::endl;
#endif
#ifdef PDAL_HAVE_PYTHON
    os << std::left
              << std::setw(name_column) << "Python" << std::right
              << std::setw(url_column) << "http://www.python.org"
              << std::setw(special_column) << ""  << std::endl;
#endif
#ifdef PDAL_HAVE_SOCI
    os << std::left
              << std::setw(name_column) << "SOCI" << std::right
              << std::setw(url_column) << "http://soci.sourceforge.net/"
              << std::setw(special_column) << ""  << std::endl;
#endif
#ifdef PDAL_HAVE_SQLITE
    os << std::left
              << std::setw(name_column) << "SQLite" << std::right
              << std::setw(url_column) << "http://www.sqlite.org/"
              << std::setw(special_column) << ""  << std::endl;
#endif
#ifdef PDAL_HAVE_POSTGRESQL
    os << std::left
              << std::setw(name_column) << "PostgreSQL" << std::right
              << std::setw(url_column) << "http://github.com/pramsey/pointcloud/"
              << std::setw(special_column) << ""  << std::endl;
#endif

    os << thdr.str() << std::endl;

    return os.str();
}

} // namespace pdal
