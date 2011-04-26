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
#include <list>
#include <cstdlib>
#include <map>

#include <string.h>


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


    m_schema_ptr = SchemaPtr(
                    xmlSchemaParse(static_cast<xmlSchemaParserCtxtPtr>(m_schema_parser_ctx.get())), 
                    SchemaDeleter());

    m_schema_valid_ctx = SchemaValidCtxtPtr(
                            xmlSchemaNewValidCtxt(static_cast<xmlSchemaPtr>(m_schema_ptr.get())),
                            SchemaValidCtxtDeleter());

    xmlSchemaSetValidErrors(static_cast<xmlSchemaValidCtxtPtr>(m_schema_valid_ctx.get()), 
                            &OCISchemaValidityError,
                            &OCISchemaValidityDebug,
                            m_global_context);
    
    int valid_schema = xmlSchemaValidateDoc(static_cast<xmlSchemaValidCtxtPtr>(m_schema_valid_ctx.get()),
                                              static_cast<xmlDocPtr>(m_doc.get()));
    
    if (valid_schema != 0)
        throw schema_error("Document did not validate against schema!");
    
    LoadSchema();
    return;
}


Schema::~Schema()
{
}


static void
print_element_names(xmlNode * a_node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = a_node; cur_node; cur_node = cur_node->next) {
        if (cur_node->type == XML_ELEMENT_NODE) {
            printf("node type: Element, name: %s\n", cur_node->name);
        }

        print_element_names(cur_node->children);
    }
}
void Schema::LoadSchema()
{
    std::map<boost::uint32_t, libpc::Dimension> layouts;
    
    xmlDocPtr doc = static_cast<xmlDocPtr>(m_doc.get());
    xmlNode* root = xmlDocGetRootElement(doc);
    // print_element_names(root);

    
    if (compare_no_case((const char*)root->name, "PointCloudSchema"))
        throw schema_error("First node of document was not named 'PointCloudSchema'");
    
    xmlNode* dimension = root->children;
    
    while(dimension != NULL)
    {
        if (dimension->type != XML_ELEMENT_NODE) 
        {
            dimension = dimension->next;
            continue;
        }

        xmlNode* properties = dimension->children;
        
        std::string name;
        boost::uint32_t size;
        boost::uint32_t position(1);
        std::string description;
        std::string interpretation;
        double offset;
        double scale;
        double minimum;
        double maximum;
        
        while(properties != NULL)
        {
            if (properties->type != XML_ELEMENT_NODE) 
            {
                properties = properties->next;
                continue;
            }
            
            if (!compare_no_case((const char*)properties->name, "name"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_error("Unable to fetch name!");
                name = std::string((const char*)n);
                xmlFree(n);
                std::cout << "Dimension name: " << name << std::endl;
            }

            if (!compare_no_case((const char*)properties->name, "size"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_error("Unable to fetch size!");
                int s = std::atoi((const char*)n);
                if (s < 1) 
                {
                    throw schema_error("Dimension size is < 1!");
                }
                xmlFree(n);
                size = static_cast<boost::uint32_t>(s);
                std::cout << "Dimension size: " << size << std::endl;
            }

            if (!compare_no_case((const char*)properties->name, "position"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_error("Unable to fetch position!");
                int p = std::atoi((const char*)n);
                if (p < 1) 
                {
                    throw schema_error("Dimension position is < 1!");
                }
                xmlFree(n);
                position = static_cast<boost::uint32_t>(p);
                std::cout << "Dimension position: " << position << std::endl;
            }
            if (!compare_no_case((const char*)properties->name, "description"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_error("Unable to fetch description!");
                description = std::string((const char*)n);
                xmlFree(n);
            }
            if (!compare_no_case((const char*)properties->name, "interpretation"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_error("Unable to fetch interpretation!");
                interpretation = std::string((const char*)n);
                xmlFree(n);
            }

            if (!compare_no_case((const char*)properties->name, "minimum"))
            {
                xmlChar* n = xmlGetProp(properties, (const xmlChar*) "value");
                if (!n) throw schema_error("Unable to fetch minimum value!");
                
                minimum = std::atof((const char*)n);
                xmlFree(n);
                std::cout << "Dimension minimum: " << minimum << std::endl;
            }

            if (!compare_no_case((const char*)properties->name, "maximum"))
            {
                xmlChar* n = xmlGetProp(properties, (const xmlChar*) "value");
                if (!n) throw schema_error("Unable to fetch maximum value!");
                
                maximum = std::atof((const char*)n);
                xmlFree(n);
                std::cout << "Dimension maximum: " << maximum << std::endl;
            }

            if (!compare_no_case((const char*)properties->name, "offset"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_error("Unable to fetch offset value!");
                
                offset = std::atof((const char*)n);
                xmlFree(n);
                std::cout << "Dimension offset: " << offset << std::endl;
            }
            if (!compare_no_case((const char*)properties->name, "scale"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_error("Unable to fetch scale value!");
                
                scale = std::atof((const char*)n);
                xmlFree(n);
                std::cout << "Dimension scale: " << scale << std::endl;
            }

            // printf("property name: %s\n", properties->name);
            properties = properties->next;
        }
        
        Dimension::DataType t = GetDimensionType(interpretation);
        Dimension::Field f = GetDimensionField(name, position);
        
        Dimension d(f, t);
        
        layouts.insert(std::pair<boost::uint32_t, libpc::Dimension>(position, d));
        dimension = dimension->next;
    }
    
}

Dimension::DataType Schema::GetDimensionType(std::string const& interpretation)
{

    // enum DataType
    // {
    //     Int8,
    //     Uint8,
    //     Int16,
    //     Uint16,
    //     Int32,
    //     Uint32,
    //     Int64,
    //     Uint64,
    //     Float,       // 32 bits
    //     Double,       // 64 bits
    //     Undefined
    // };


    if (!compare_no_case(interpretation.c_str(), "int8_t") || 
        !compare_no_case(interpretation.c_str(), "int8"))
        return Dimension::Int8;

    if (!compare_no_case(interpretation.c_str(), "uint8_t") || 
        !compare_no_case(interpretation.c_str(), "uint8"))
        return Dimension::Uint8;

    if (!compare_no_case(interpretation.c_str(), "int16_t") || 
        !compare_no_case(interpretation.c_str(), "int16"))
        return Dimension::Int16;

    if (!compare_no_case(interpretation.c_str(), "uint16_t") || 
        !compare_no_case(interpretation.c_str(), "uint16"))
        return Dimension::Uint16;
    
    
    if (!compare_no_case(interpretation.c_str(), "int32_t") || 
        !compare_no_case(interpretation.c_str(), "int32"))
        return Dimension::Int32;

    if (!compare_no_case(interpretation.c_str(), "uint32_t") || 
        !compare_no_case(interpretation.c_str(), "uint32"))
        return Dimension::Uint32;

    if (!compare_no_case(interpretation.c_str(), "int64_t") || 
        !compare_no_case(interpretation.c_str(), "int64"))
        return Dimension::Int64;

    if (!compare_no_case(interpretation.c_str(), "uint64_t") || 
        !compare_no_case(interpretation.c_str(), "uint64"))
        return Dimension::Uint64;

    if (!compare_no_case(interpretation.c_str(), "float"))
        return Dimension::Float;

    if (!compare_no_case(interpretation.c_str(), "double"))
        return Dimension::Double;


    return Dimension::Undefined;
}

Dimension::Field Schema::GetDimensionField(std::string const& name, boost::uint32_t position)
{
    
    if (name.size() == 0)
    {
        // Yes, this is scary.  What else can we do?  The user didn't give us a 
        // name to map to, so we'll just assume the positions are the same as 
        // our dimension positions
        for (unsigned int i = 1; i < Dimension::Field_INVALID; ++i)
        {
            if (i == position)
                return static_cast<Dimension::Field>(i);
        }
    }
    
    if (!compare_no_case(name.c_str(), "X"))
        return Dimension::Field_X;

    if (!compare_no_case(name.c_str(), "Y"))
        return Dimension::Field_Y;

    if (!compare_no_case(name.c_str(), "Z"))
        return Dimension::Field_Z;

    if (!compare_no_case(name.c_str(), "Intensity"))
        return Dimension::Field_Intensity;

    if (!compare_no_case(name.c_str(), "Return Number"))
        return Dimension::Field_ReturnNumber;

    if (!compare_no_case(name.c_str(), "Number of Returns"))
        return Dimension::Field_NumberOfReturns;

    if (!compare_no_case(name.c_str(), "Number of Returns"))
        return Dimension::Field_NumberOfReturns;

    if (!compare_no_case(name.c_str(), "Scan Direction"))
        return Dimension::Field_ScanDirectionFlag;

    if (!compare_no_case(name.c_str(), "Flightline Edge"))
        return Dimension::Field_EdgeOfFlightLine;

    if (!compare_no_case(name.c_str(), "Classification"))
        return Dimension::Field_Classification;

    if (!compare_no_case(name.c_str(), "Scan Angle Rank"))
        return Dimension::Field_ScanAngleRank;

    if (!compare_no_case(name.c_str(), "User Data"))
        return Dimension::Field_UserData;

    if (!compare_no_case(name.c_str(), "Point Source ID"))
        return Dimension::Field_PointSourceId;

    if (!compare_no_case(name.c_str(), "Time"))
        return Dimension::Field_Time;

    if (!compare_no_case(name.c_str(), "Red"))
        return Dimension::Field_Red;

    if (!compare_no_case(name.c_str(), "Green"))
        return Dimension::Field_Green;

    if (!compare_no_case(name.c_str(), "Blue"))
        return Dimension::Field_Blue;


    return Dimension::Field_INVALID;
}

} } } // namespaces
