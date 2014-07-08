/******************************************************************************
* Copyright (c) 2014, Hobu Inc.
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

#include <memory>

#include "pdal/Metadata.hpp"
#include "pdal/Schema.hpp"
#include "pdal/RawPtBuf.hpp"

namespace pdal
{

// This provides a context for processing a set of points and allows the library
// to be used to process multiple point sets simultaneously.
class PointContext
{
private:
    // Provides information on the dimensions/layout inforamtion for the points.
    SchemaPtr m_schema;
    // Provides storage for the point data.
    RawPtBufPtr m_ptBuf;
    // Metadata storage;
    MetadataPtr m_metadata;

public:
    PointContext() : m_schema(new Schema), m_ptBuf(new RawPtBuf(m_schema)),
        m_metadata(new Metadata)
    {}

    Schema *schema() const
        { return m_schema.get(); }
    RawPtBuf *rawPtBuf() const
        { return m_ptBuf.get(); }
    MetadataNode metadata()
        { return m_metadata->getNode(); }
    SpatialReference spatialRef() const
    {
        MetadataNode m = m_metadata->m_private.findChild("spatialreference");
        SpatialReference sref;
        sref.setWKT(m.value());
        return sref;
    }
    void setSpatialRef(const SpatialReference& sref)
    {
        MetadataNode mp = m_metadata->m_private;
        mp.addOrUpdate("spatialreference", sref.getRawWKT());
    }
};

} //namespace

