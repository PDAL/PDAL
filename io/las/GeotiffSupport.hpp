/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
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

#pragma once

#include <pdal/pdal_internal.hpp>

// GDAL
#include <geo_normalize.h>
#include <ogr_spatialref.h>

// See http://lists.osgeo.org/pipermail/gdal-dev/2013-November/037429.html
#define CPL_SERV_H_INCLUDED

#include <string>
#include <stdexcept>

struct GTIFS;
struct StTiff;

namespace pdal
{

class PDAL_DLL GeotiffSupport
{
public:
    GeotiffSupport() : m_gtiff(0), m_tiff(0)
    {}
    ~GeotiffSupport();

    void resetTags();
    void setShortKeys(int tag, void *data, int size);
    void setDoubleKeys(int tag, void *data, int size);
    void setAsciiKeys(int tag, void *data, int size);
    size_t getKey(int tag, int *count, void **data_ptr) const;
    void setTags();

    std::string getWkt(bool horizOnly, bool pretty) const;
    void setWkt(const std::string&);

    std::string getText() const;

private:
    void rebuildGTIF();

    GTIF *m_gtiff;
    StTiff *m_tiff;
};

} // namespace pdal
