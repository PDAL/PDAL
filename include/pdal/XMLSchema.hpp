/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#define PDAL_XML_SCHEMA_VERSION "1.3"

#ifndef PDAL_HAVE_LIBXML2
#error "Including XMLSchema.hpp without libxml2 - build problem."
#endif

#include <pdal/Dimension.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/PointLayout.hpp>

#include <string>
#include <stdarg.h>
#include <vector>

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xinclude.h>
#include <libxml/xmlIO.h>
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>

namespace pdal
{

void OCISchemaGenericErrorHandler(void * ctx, const char* message, ...);
void OCISchemaStructuredErrorHandler(void * userData, xmlErrorPtr error);

class XMLSchema;

struct XMLDim
{
    friend class XMLSchema;

public:
    XMLDim() : m_min(0.0), m_max(0.0)
    {}
    XMLDim(const DimType& dim, const std::string& name) :
        m_name(name), m_min(0.0), m_max(0.0), m_dimType(dim)
    {}

    std::string m_name;
    std::string m_description;
    uint32_t m_position;
    double m_min;
    double m_max;
    DimType m_dimType;
};
typedef std::vector<XMLDim> XMLDimList;
inline bool operator < (const XMLDim& d1, const XMLDim& d2)
    { return d1.m_position < d2.m_position; }

class PDAL_DLL XMLSchema
{
public:
    XMLSchema(std::string xml, std::string xsd = "",
        Orientation::Enum orientation = Orientation::PointMajor);
    XMLSchema(const XMLDimList& dims, MetadataNode m = MetadataNode(),
        Orientation::Enum orientation = Orientation::PointMajor);
    XMLSchema(const PointLayoutPtr& pointTable, MetadataNode m = MetadataNode(),
        Orientation::Enum orientation = Orientation::PointMajor);
    XMLSchema() : m_orientation(Orientation::PointMajor)
    {}

    ~XMLSchema()
        { xmlCleanupParser(); }

    std::string xml() const;
    DimTypeList dimTypes() const;
    XMLDimList xmlDims() const
        { return m_dims; }

    MetadataNode getMetadata() const
        { return m_metadata;}
    void setId(const std::string& name, Dimension::Id::Enum id)
        { xmlDim(name).m_dimType.m_id = id; }
    void setXForm(Dimension::Id::Enum id, XForm xform)
        { xmlDim(id).m_dimType.m_xform = xform; }
    XForm xForm(Dimension::Id::Enum id) const
        { return xmlDim(id).m_dimType.m_xform; }
    void setOrientation(Orientation::Enum orientation)
        { m_orientation = orientation; }
    Orientation::Enum orientation() const
        { return m_orientation; }

private:
    Orientation::Enum m_orientation;
    XMLDimList m_dims;
    void* m_global_context;
    MetadataNode m_metadata;

    XMLDim& xmlDim(Dimension::Id::Enum id);
    const XMLDim& xmlDim(Dimension::Id::Enum id) const;
    XMLDim& xmlDim(const std::string& name);
    xmlDocPtr init(const std::string& xml, const std::string& xsd);
    bool validate(xmlDocPtr doc, const std::string& xsd);
    std::string remapOldNames(const std::string& input);
    bool loadMetadata(xmlNode *startNode, MetadataNode& input);
    bool load(xmlDocPtr doc);
    void writeXml(xmlTextWriterPtr w) const;
};

} // namespace pdal

