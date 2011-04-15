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



namespace libpc { namespace drivers { namespace oci {


void OCISchemaStructuredErrorHandler 
    (void * userData, xmlErrorPtr error)
{
    std::ostringstream oss;
    
    oss << "XML error: '" << error->message <<"' ";
    oss << "on line " << error->line;
    throw schema_error(oss.str());
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
    throw schema_error(oss.str());
    
}


Schema::Schema(std::string xml, std::string xmlschema)

{

    LIBXML_TEST_VERSION


    
    xmlSetGenericErrorFunc(NULL, (xmlGenericErrorFunc) &OCISchemaGenericErrorHandler);
    xmlSetStructuredErrorFunc(NULL, (xmlStructuredErrorFunc) & OCISchemaStructuredErrorHandler);

    // No network access
    // http://xmlsoft.org/html/libxml-parser.html#xmlParserOption
    m_doc = xmlReadMemory(xml.c_str(), xml.size(), "noname.xml", NULL, XML_PARSE_NONET);

    
    
    return;
}


Schema::~Schema()
{
}



} } } // namespaces
