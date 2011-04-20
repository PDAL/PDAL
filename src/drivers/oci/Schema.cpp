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

#include <libpc/drivers/oci/Schema.hpp>
#include <libpc/exceptions.hpp>
#include <libpc/Utils.hpp>

#include <sstream>
#include <iostream>

struct XMLDocDeleter
{
   template <typename T>
   void operator()(T* ptr)
   {
       ::xmlFreeDoc(ptr);
   }
};

struct SchemaParserCtxDeleter
{
   template <typename T>
   void operator()(T* ptr)
   {
       ::xmlSchemaFreeParserCtxt(ptr);
   }
};

struct SchemaDeleter
{
   template <typename T>
   void operator()(T* ptr)
   {
       ::xmlSchemaFree(ptr);
   }
};

struct SchemaValidCtxtDeleter
{
   template <typename T>
   void operator()(T* ptr)
   {
       ::xmlSchemaFreeValidCtxt(ptr);
   }
};



namespace libpc { namespace drivers { namespace oci {


void OCISchemaStructuredErrorHandler 
    (void * userData, xmlErrorPtr error)
{
    std::ostringstream oss;
    
    oss << "XML error: '" << error->message <<"' ";
    
    if (error->str1)
        oss << " extra info1: '" << error->str1 << "' ";
    if (error->str2)
        oss << " extra info2: '" << error->str2 << "' ";
    if (error->str3)
        oss << " extra info3: '" << error->str3 << "' ";
    oss << "on line " << error->line;
    
    if (error->ctxt)
    {
        xmlParserCtxtPtr ctxt = (xmlParserCtxtPtr) error->ctxt;
        xmlParserInputPtr input = ctxt->input;
        
        xmlParserPrintFileContext(input);
    }
    throw schema_error(oss.str());
}

void OCISchemaParserStructuredErrorHandler 
    (void * userData, xmlErrorPtr error)
{
    std::ostringstream oss;
    
    oss << "Schema parsing error: '" << error->message <<"' ";
    oss << "on line " << error->line;
    throw schema_parsing_error(oss.str());
}

void OCISchemaValidationStructuredErrorHandler 
    (void * userData, xmlErrorPtr error)
{
    std::ostringstream oss;
    
    oss << "Schema validation error: '" << error->message <<"' ";
    oss << "on line " << error->line;
    throw schema_validation_error(oss.str());
}

void OCISchemaValidityError 
    (void * ctx, const char* message, ... )
{
    
    const int ERROR_MESSAGE_SIZE = 256;
    char error[ERROR_MESSAGE_SIZE];
    va_list arg_ptr;

    va_start(arg_ptr, message);
    vsnprintf(error, ERROR_MESSAGE_SIZE, message, arg_ptr);
    va_end(arg_ptr);

    
    std::ostringstream oss;
    
    oss << "Schema valididy error: '" << error <<"' ";
    throw schema_validation_error(oss.str());
    
}

void OCISchemaValidityDebug 
    (void * ctx, const char* message, ... )
{
    
    const int ERROR_MESSAGE_SIZE = 256;
    char error[ERROR_MESSAGE_SIZE];
    va_list arg_ptr;

    va_start(arg_ptr, message);
    vsnprintf(error, ERROR_MESSAGE_SIZE, message, arg_ptr);
    va_end(arg_ptr);

    
    std::ostringstream oss;
    
    oss << "Schema validity debug: '" << error <<"' ";
    std::cout << oss.str() << std::endl;
    
}


void OCISchemaGenericErrorHandler 
    (void * ctx, const char* message, ... )
{
    
    const int ERROR_MESSAGE_SIZE = 256;
    char error[ERROR_MESSAGE_SIZE];
    va_list arg_ptr;

    va_start(arg_ptr, message);
    vsnprintf(error, ERROR_MESSAGE_SIZE, message, arg_ptr);
    va_end(arg_ptr);

    
    std::ostringstream oss;
    
    oss << "Generic error: '" << error <<"' ";
    throw schema_generic_error(oss.str());
    
}



// XML_PARSE_NONET No network access
// http://xmlsoft.org/html/libxml-parser.html#xmlParserOption

Schema::Schema(std::string const& xml, std::string const &xsd)
: m_doc_options(XML_PARSE_NONET)
{
    if (xml.size() == 0) throw schema_generic_error("Inputted XML has no size, is there data there?");
    if (xsd.size() == 0) throw schema_generic_error("Inputted XSD has no size, is there data there?");


    LIBXML_TEST_VERSION
    
    xmlSetGenericErrorFunc(m_global_context, (xmlGenericErrorFunc) &OCISchemaGenericErrorHandler);
    xmlSetStructuredErrorFunc(m_global_context, (xmlStructuredErrorFunc) & OCISchemaStructuredErrorHandler);


    m_doc = DocPtr(
                   xmlReadMemory(xml.c_str(), xml.size(), NULL, NULL, m_doc_options), 
                   XMLDocDeleter());
                   
    m_schema_doc = DocPtr(
                        xmlReadMemory(xsd.c_str(), xsd.size(), NULL, NULL, m_doc_options), 
                         XMLDocDeleter());

    m_schema_parser_ctx = SchemaParserCtxtPtr(
                            xmlSchemaNewDocParserCtxt(static_cast<xmlDocPtr>(m_schema_doc.get())),
                            SchemaParserCtxDeleter());
    
    xmlSchemaSetParserStructuredErrors(static_cast<xmlSchemaParserCtxtPtr>(m_schema_parser_ctx.get()), 
                                        &OCISchemaParserStructuredErrorHandler,
                                        m_global_context);


    m_schema = SchemaPtr(
                    xmlSchemaParse(static_cast<xmlSchemaParserCtxtPtr>(m_schema_parser_ctx.get())), 
                    SchemaDeleter());

    m_schema_valid_ctx = SchemaValidCtxtPtr(
                            xmlSchemaNewValidCtxt(static_cast<xmlSchemaPtr>(m_schema.get())),
                            SchemaValidCtxtDeleter());

    xmlSchemaSetValidErrors(static_cast<xmlSchemaValidCtxtPtr>(m_schema_valid_ctx.get()), 
                            &OCISchemaValidityError,
                            &OCISchemaValidityDebug,
                            m_global_context);
    
    int valid_schema = xmlSchemaValidateDoc(static_cast<xmlSchemaValidCtxtPtr>(m_schema_valid_ctx.get()),
                                              static_cast<xmlDocPtr>(m_doc.get()));
    
    if (valid_schema != 0)
        throw schema_error("Document did not validate against schema!");
    return;
}


Schema::~Schema()
{
}



} } } // namespaces
