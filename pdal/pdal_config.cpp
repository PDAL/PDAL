/******************************************************************************
 * Copyright (c) 2008, Mateusz Loskot
 * Copyright (c) 2010, Frank Warmerdam
 * Copyright (c) 2018, Hobu Inc.
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
#include <pdal/pdal_features.hpp>

#include <sstream>
#include <iomanip>

#include <pdal/gitsha.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wfloat-equal"
#include <gdal.h>
#pragma clang diagnostic pop

#ifdef PDAL_HAVE_LIBXML2
#include <libxml/xmlversion.h>
#endif

#include <pdal/util/Utils.hpp>

namespace pdal
{
namespace Config
{

bool hasFeature(Feature f)
{
    bool enabled = false;
    switch (f)
    {
    case Feature::LASZIP:
#ifdef PDAL_HAVE_LASZIP
        enabled = true;
#endif
        break;
    case Feature::LAZPERF:
#ifdef PDAL_HAVE_LAZPERF
        enabled = true;
#endif
        break;
    case Feature::ZSTD:
#ifdef PDAL_HAVE_ZSTD
        enabled = true;
#endif
        break;
    case Feature::ZLIB:
#ifdef PDAL_HAVE_ZLIB
        enabled = true;
#endif
        break;
    case Feature::LZMA:
#ifdef PDAL_HAVE_LZMA
        enabled = true;
#endif
        break;
    case Feature::LIBXML2:
#ifdef PDAL_HAVE_LIBXML2
        enabled = true;
#endif
        break;
    default:
        break;
    }
    return enabled;
}


int versionMajor()
{
    return PDAL_VERSION_MAJOR;
}


int versionMinor()
{
    return PDAL_VERSION_MINOR;
}


int versionPatch()
{
    return PDAL_VERSION_PATCH;
}


std::string versionString()
{
    return std::string(PDAL_VERSION);
}

int versionInteger()
{
    return PDAL_VERSION_INTEGER;
}

std::string sha1()
{
	return g_GIT_SHA1;
}


/// Tell the user a bit about PDAL's compilation
std::string fullVersionString()
{
    std::ostringstream os;

    std::string sha = sha1();
    if (!Utils::iequals(sha, "Release"))
        sha = sha.substr(0,6);

    os << PDAL_VERSION << " (git-version: " << sha << ")";

    return os.str();
}


std::string debugInformation()
{
    Utils::screenWidth();
    std::string headline(Utils::screenWidth(), '-');

    std::ostringstream os;

    os << headline << std::endl;
    os << "PDAL debug information" << std::endl ;
    os << headline << std::endl << std::endl;

    os << "Version information" << std::endl;
    os << headline << std::endl;
    os << "(" << fullVersionString() << ")" << std::endl;
    os << std::endl;

    os << "Debug build status" << std::endl;
    os << headline << std::endl;
    os << PDAL_BUILD_TYPE << std::endl << std::endl;

    os << "Enabled libraries" << std::endl;
    os << headline << std::endl << std::endl;

    os << "GDAL (" << GDALVersionInfo("RELEASE_NAME") << ") - " <<
        "http://www.gdal.org" << std::endl;

#ifdef PDAL_HAVE_LIBXML2
    os << "libxml (" << LIBXML_DOTTED_VERSION << ") - " <<
              "http://www.xmlsoft.org/" << std::endl;
#endif

    return os.str();
}

std::string pluginInstallPath()
{
    return PDAL_PLUGIN_INSTALL_PATH;
}

} // namespace Config
} // namespace pdal
