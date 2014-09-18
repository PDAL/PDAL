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

#include <pdal/Dimension.hpp>
#include <pdal/Metadata.hpp>

#include <string>
#include <stdarg.h>
//#include <functional>
#include <vector>

#ifdef PDAL_HAVE_LIBXML2
#include <libxml/parser.h>
#include <libxml/xmlschemas.h>

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xinclude.h>
#include <libxml/xmlIO.h>
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>
#endif

#include <boost/shared_ptr.hpp>
//#include <boost/concept_check.hpp>
//#include <boost/function.hpp>

namespace pdal
{
namespace schema
{

#ifdef PDAL_HAVE_LIBXML2

void OCISchemaGenericErrorHandler(void * ctx, const char* message, ...);
void OCISchemaStructuredErrorHandler(void * userData, xmlErrorPtr error);
#endif

class schema_error : public pdal_error
{
public:
    schema_error(std::string const& msg)
        : pdal_error(msg)
    {}
};

class schema_loading_error : public schema_error
{
public:
    schema_loading_error(std::string const& msg)
        : schema_error(msg)
    {}
};

class schema_writing_error : public schema_error
{
public:
    schema_writing_error(std::string const& msg)
        : schema_error(msg)
    {}
};


class schema_validation_error : public schema_error
{
public:
    schema_validation_error(std::string const& msg)
        : schema_error(msg)
    {}
};

class schema_parsing_error : public schema_error
{
public:
    schema_parsing_error(std::string const& msg)
        : schema_error(msg)
    {}
};

class schema_generic_error : public schema_error
{
public:
    schema_generic_error(std::string const& msg)
        : schema_error(msg)
    {}
};


// We're going to put all of our libxml2 primatives into shared_ptrs
// that have custom deleters that clean up after themselves so we
// have a good chance at having clean exception-safe code
typedef boost::shared_ptr<void> DocPtr;
typedef boost::shared_ptr<void> SchemaParserCtxtPtr;
typedef boost::shared_ptr<void> SchemaPtr;
typedef boost::shared_ptr<void> SchemaValidCtxtPtr;
typedef boost::shared_ptr<void> TextWriterPtr;
typedef boost::shared_ptr<void> BufferPtr;
typedef boost::shared_ptr<void> CharPtr;

struct Scale
{
    Scale() : m_scale(1.0), m_offset(0.0) {};
    double m_scale;
    double m_offset;
};

struct XYZScale
{
    Scale m_x;
    Scale m_y;
    Scale m_z;
};

struct DimInfo
{
    DimInfo() : m_min(0.0), m_max(0.0), m_scale(1.0), m_offset(0.0) {};
    std::string m_name;
    std::string m_description;
    uint32_t m_position;
    Dimension::Type::Enum m_type;
    double m_min;
    double m_max;
    double m_scale;
    double m_offset;
    Dimension::Id::Enum m_id;
};
typedef std::vector<DimInfo> DimInfoList;
inline bool operator < (const DimInfo& d1, const DimInfo& d2)
    { return d1.m_position < d2.m_position; }

struct XMLSchema
{
    XMLSchema() : m_orientation(Orientation::PointMajor) {};
    Orientation::Enum m_orientation;
    DimInfoList m_dims;
    XYZScale m_scale;  // To support quick access.

    Dimension::IdList dims() const
    {
        Dimension::IdList ids;
        for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
        {
            assert(di->m_id != Dimension::Id::Unknown);
            ids.push_back(di->m_id);
        }
        return ids;
    }
    std::vector<Dimension::Type::Enum> types() const
    {
        std::vector<Dimension::Type::Enum> types;
        for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
            types.push_back(di->m_type);
        return types;
    }
};

class PDAL_DLL Reader
{
public:
    Reader(std::string xml, std::string xsd = "");
    Reader(std::istream* xml, std::istream* xsd);
    ~Reader();

    XMLSchema schema() const
        { return m_schema; }
protected:
    void Initialize();
    void Load();

private:
#ifdef PDAL_HAVE_LIBXML2
    pdal::Metadata LoadMetadata(xmlNode* node);
    std::string remapOldNames(std::string const& input);

    DocPtr m_doc;
    DocPtr m_schema_doc;

    SchemaParserCtxtPtr m_schema_parser_ctx;
    SchemaPtr m_schema_ptr;
    SchemaValidCtxtPtr m_schema_valid_ctx;

    xmlParserOption m_doc_options;
#endif

    void *m_global_context;
    std::string m_xml;
    std::string m_xsd;
    uint32_t m_field_position;
    XMLSchema m_schema;

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented;
};


class PDAL_DLL Writer
{
public:
    Writer(const Dimension::IdList& ids,
        const std::vector<Dimension::Type::Enum>& types, 
        Orientation::Enum orientation= Orientation::PointMajor) :
            m_dims(ids), m_types(types), m_orientation(orientation)
        {}
    void setMetadata(MetadataNode& m)
        { m_metadata = m; }
    std::string getXML();

private:
    void write(TextWriterPtr w);
    void writeSchema(TextWriterPtr w);

    void* m_global_context;
    Dimension::IdList m_dims;
    std::vector<Dimension::Type::Enum> m_types;
    MetadataNode m_metadata;
    Orientation::Enum m_orientation;

    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented;
};

} // namespace schema
} // namespace pdal

