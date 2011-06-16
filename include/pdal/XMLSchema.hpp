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

#ifndef INCLUDED_XMLSCHEMA_HPP
#define INCLUDED_XMLSCHEMA_HPP

#include <pdal/pdal.hpp>
#include <pdal/Schema.hpp>
#include <pdal/Utils.hpp>
#include <pdal/SchemaLayout.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/DimensionLayout.hpp>
#include <pdal/exceptions.hpp>

#include <string>
#include <stdarg.h>
#include <functional>

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xinclude.h>
#include <libxml/xmlIO.h>
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>

#include <boost/shared_ptr.hpp>
#include <boost/concept_check.hpp>
#include <boost/function.hpp>

namespace pdal { namespace schema {


void OCISchemaGenericErrorHandler (void * ctx, const char* message, ...);
void OCISchemaStructuredErrorHandler (void * userData, xmlErrorPtr error);

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

class PDAL_DLL Reader
{
public:
    Reader(std::string const& xml, std::string const& xmlschema);
    Reader(std::istream* xml, std::istream* schema);
    ~Reader();

    inline pdal::Schema getSchema() { return m_schema; }


protected:

    void Initialize();
    void Load();
    Dimension::DataType GetDimensionType(std::string const& interpretation);
    Dimension::Field GetDimensionField(std::string const& name, boost::uint32_t position);
    
private:
    
    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented;




    DocPtr m_doc;
    DocPtr m_schema_doc;
    
    SchemaParserCtxtPtr m_schema_parser_ctx;
    SchemaPtr m_schema_ptr;
    SchemaValidCtxtPtr m_schema_valid_ctx;
    
    xmlParserOption m_doc_options;

    
    void* m_global_context;
    pdal::Schema m_schema;
    
    std::string m_xml;
    std::string m_xsd;
    
    boost::uint32_t m_field_position;
    
    

};


class PDAL_DLL Writer
{
public:
    Writer(pdal::Schema const& schema);
    ~Writer() {};

    std::string getXML();


protected:


    
private:
    
    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented;


    

    void write(TextWriterPtr w);
    void writeSchema(TextWriterPtr w);
    void* m_global_context;
    pdal::Schema const& m_schema;
    
    

};



}} // namespaces

#endif
