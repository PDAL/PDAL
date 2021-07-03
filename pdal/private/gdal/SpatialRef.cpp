/******************************************************************************
* Copyright (c) 2020, Hobu Inc.
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

#include <string>

#include <ogr_srs_api.h>
#include <ogr_api.h>
#include <cpl_conv.h>

#include "SpatialRef.hpp"

namespace pdal
{
namespace gdal
{

SpatialRef::SpatialRef()
{
    newRef(OSRNewSpatialReference(""));
}


SpatialRef::SpatialRef(const std::string& srs)
{
    newRef(OSRNewSpatialReference(""));
    if (OSRSetFromUserInput(get(), srs.data()) != OGRERR_NONE)
        m_ref.reset();
}


void SpatialRef::setFromLayer(OGRLayerH layer)
{
    if (layer)
    {
        OGRSpatialReferenceH s = OGR_L_GetSpatialRef(layer);
        if (s)
        {
            OGRSpatialReferenceH clone = OSRClone(s);
            newRef(clone);
        }
    }
}


SpatialRef::operator bool () const
{
    return m_ref.get() != NULL;
}


OGRSpatialReferenceH SpatialRef::get() const
{
    return m_ref.get();
}


std::string SpatialRef::wkt() const
{
    std::string output;

    if (m_ref.get())
    {
        char *pszWKT = NULL;
        OSRExportToWkt(m_ref.get(), &pszWKT);
        bool valid = (bool)*pszWKT;
        output = pszWKT;
        CPLFree(pszWKT);
    }
    return output;
}


bool SpatialRef::empty() const
{
    return wkt().empty();
}

void SpatialRef::newRef(void *v)
{
    m_ref = RefPtr(v, [](void* t){ OSRDestroySpatialReference(t); } );
}

} // namespace gdal
} // namespace pdal
