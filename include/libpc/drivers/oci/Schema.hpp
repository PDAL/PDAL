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

#ifndef INCLUDED_DRIVERS_OCI_SCHEMA_HPP
#define INCLUDED_DRIVERS_OCI_SCHEMA_HPP

#include <libpc/libpc.hpp>
#include <libpc/Schema.hpp>
#include <libpc/SchemaLayout.hpp>
#include <libpc/Dimension.hpp>
#include <libpc/DimensionLayout.hpp>

#include <libpc/drivers/oci/Common.hpp>
#include <libpc/drivers/oci/Reader.hpp>

#include <string>
#include <stdarg.h>
#include <functional>

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xinclude.h>
#include <libxml/xmlIO.h>

#include <boost/shared_ptr.hpp>
#include <boost/concept_check.hpp>
#include <boost/function.hpp>

namespace libpc { namespace drivers { namespace oci {


void OCISchemaGenericErrorHandler (void * ctx, const char* message, ...);
void OCISchemaStructuredErrorHandler (void * userData, xmlErrorPtr error);

class LIBPC_DLL Schema
{
public:
    Schema(std::string const& xml, std::string const& xmlschema);
    ~Schema();



protected:

    void LoadSchema();
    Dimension::DataType GetDimensionType(std::string const& interpretation);
    Dimension::Field GetDimensionField(std::string const& name, boost::uint32_t position);
    
private:
    
    Schema& operator=(const Schema&); // not implemented
    Schema(const Schema&); // not implemented;
    
    // We're going to put all of our libxml2 primatives into shared_ptrs 
    // that have custom deleters that clean up after themselves so we 
    // have a good chance at having clean exception-safe code
    
    typedef boost::shared_ptr<void> DocPtr;
    typedef boost::shared_ptr<void> SchemaParserCtxtPtr;    
    typedef boost::shared_ptr<void> SchemaPtr;
    typedef boost::shared_ptr<void> SchemaValidCtxtPtr;

    DocPtr m_doc;
    DocPtr m_schema_doc;
    
    SchemaParserCtxtPtr m_schema_parser_ctx;
    SchemaPtr m_schema_ptr;
    SchemaValidCtxtPtr m_schema_valid_ctx;
    
    xmlParserOption m_doc_options;
    
    void* m_global_context;
    libpc::Schema m_schema;
    
    
    

};



} } } // namespaces

#endif
