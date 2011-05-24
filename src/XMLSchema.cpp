/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com *
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

#include <libpc/XMLSchema.hpp>
#include <libpc/exceptions.hpp>
#include <libpc/Utils.hpp>

#include <sstream>
#include <iostream>
#include <list>
#include <cstdlib>
#include <map>
#include <algorithm>

#include <string.h>
#include <stdlib.h>


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

struct WriterDeleter
{
   template <typename T>
   void operator()(T* ptr)
   {
       ::xmlFreeTextWriter(ptr);
   }
};

struct BufferDeleter
{
   template <typename T>
   void operator()(T* ptr)
   {
       ::xmlBufferFree(ptr);
   }
};

struct xmlCharDeleter
{
   template <typename T>
   void operator()(T* ptr)
   {
       ::xmlFree(ptr);
   }
};


static bool sort_dimensions(libpc::DimensionLayout const& a, libpc::DimensionLayout const& b)
{
   return a < b;
}

namespace libpc { namespace schema {


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


void Reader::Initialize() 
{
    if (m_xml.size() == 0) throw schema_generic_error("Inputted XML has no size, is there data there?");
    if (m_xsd.size() == 0) throw schema_generic_error("Inputted XSD has no size, is there data there?");


    LIBXML_TEST_VERSION

    xmlSetGenericErrorFunc(m_global_context, (xmlGenericErrorFunc) &OCISchemaGenericErrorHandler);
    xmlSetStructuredErrorFunc(m_global_context, (xmlStructuredErrorFunc) & OCISchemaStructuredErrorHandler);


    m_doc = DocPtr(
                   xmlReadMemory(m_xml.c_str(), m_xml.size(), NULL, NULL, m_doc_options),
                   XMLDocDeleter());

    m_schema_doc = DocPtr(
                        xmlReadMemory(m_xsd.c_str(), m_xsd.size(), NULL, NULL, m_doc_options),
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
    
}

Reader::Reader(std::string const& xml, std::string const &xsd)
: m_doc_options(XML_PARSE_NONET)
{
    
    m_xml = xml;
    m_xsd = xsd;
    Initialize();
    Load();
    return;
}

Reader::Reader(std::istream* xml, std::istream *xsd) : m_doc_options(XML_PARSE_NONET)
{
    
    if (!xml)
        throw schema_generic_error("libpc::schema::Reader: xml istream pointer was null!");
    
    std::istream::pos_type size;

    std::vector<char> data;
    xml->seekg(0, std::ios::end);
    size = xml->tellg();
    data.resize(static_cast<std::vector<char>::size_type>(size));
    xml->seekg (0, std::ios::beg);
    xml->read (&data.front(), size);
    xml->seekg (0, std::ios::beg);

    m_xml = std::string(&data[0], data.size());

    if (xsd)
    {

        std::istream::pos_type size;

        std::vector<char> data;
        xsd->seekg(0, std::ios::end);
        size = xsd->tellg();
        data.resize(static_cast<std::vector<char>::size_type>(size));
        xsd->seekg (0, std::ios::beg);
        xsd->read (&data.front(), size);
        xsd->seekg(0, std::ios::end);

        m_xsd = std::string(&data[0], data.size());
    }

    Initialize();
    Load();
}

Reader::~Reader()
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



void Reader::Load()
{
    std::vector<libpc::DimensionLayout> layouts;

    xmlDocPtr doc = static_cast<xmlDocPtr>(m_doc.get());
    xmlNode* root = xmlDocGetRootElement(doc);
    // print_element_names(root);


    if (compare_no_case((const char*)root->name, "PointCloudSchema"))
        throw schema_loading_error("First node of document was not named 'PointCloudSchema'");

    xmlNode* dimension = root->children;


    while(dimension != NULL)
    {
        // printf("node name: %s\n", (const char*)dimension->name);
        if (dimension->type != XML_ELEMENT_NODE || compare_no_case((const char*)dimension->name, "dimension"))
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
                CharPtr n = CharPtr(
                                xmlNodeListGetString(doc, properties->children, 1),
                                xmlCharDeleter());                
                            // xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch name!");
                name = std::string((const char*)n.get());
                // xmlFree(n);
                // std::cout << "Dimension name: " << name << std::endl;
            }

            if (!compare_no_case((const char*)properties->name, "size"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch size!");
                int s = std::atoi((const char*)n);
                if (s < 1)
                {
                    throw schema_loading_error("Dimension size is < 1!");
                }
                xmlFree(n);
                size = static_cast<boost::uint32_t>(s);
                // std::cout << "Dimension size: " << size << std::endl;
            }

            if (!compare_no_case((const char*)properties->name, "position"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch position!");
                int p = std::atoi((const char*)n);
                if (p < 1)
                {
                    throw schema_loading_error("Dimension position is < 1!");
                }
                xmlFree(n);
                position = static_cast<boost::uint32_t>(p);
                // std::cout << "Dimension position: " << position << std::endl;
            }
            if (!compare_no_case((const char*)properties->name, "description"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch description!");
                description = std::string((const char*)n);
                xmlFree(n);
            }
            if (!compare_no_case((const char*)properties->name, "interpretation"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch interpretation!");
                interpretation = std::string((const char*)n);
                xmlFree(n);
            }

            if (!compare_no_case((const char*)properties->name, "minimum"))
            {
                xmlChar* n = xmlGetProp(properties, (const xmlChar*) "value");
                if (!n) throw schema_loading_error("Unable to fetch minimum value!");

                minimum = std::atof((const char*)n);
                xmlFree(n);
                // std::cout << "Dimension minimum: " << minimum << std::endl;
            }

            if (!compare_no_case((const char*)properties->name, "maximum"))
            {
                xmlChar* n = xmlGetProp(properties, (const xmlChar*) "value");
                if (!n) throw schema_loading_error("Unable to fetch maximum value!");

                maximum = std::atof((const char*)n);
                xmlFree(n);
                // std::cout << "Dimension maximum: " << maximum << std::endl;
            }

            if (!compare_no_case((const char*)properties->name, "offset"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch offset value!");

                offset = std::atof((const char*)n);
                xmlFree(n);
                // std::cout << "Dimension offset: " << offset << std::endl;
            }
            if (!compare_no_case((const char*)properties->name, "scale"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch scale value!");

                scale = std::atof((const char*)n);
                xmlFree(n);
                // std::cout << "Dimension scale: " << scale << std::endl;
            }

            // printf("property name: %s\n", properties->name);
            properties = properties->next;
        }

        Dimension::DataType t = GetDimensionType(interpretation);
        Dimension::Field f = GetDimensionField(name, position);

        Dimension d(f, t);
        DimensionLayout l(d);
        l.setPosition(position);
        layouts.push_back(l);

        dimension = dimension->next;
    }

    std::sort(layouts.begin(), layouts.end(), sort_dimensions);

    std::vector<DimensionLayout>::const_iterator i;
    for (i = layouts.begin(); i!= layouts.end(); ++i)
    {
        m_schema.addDimension(i->getDimension());
    }

    // m_schema.dump();
}

Dimension::DataType Reader::GetDimensionType(std::string const& interpretation)
{

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

Dimension::Field Reader::GetDimensionField(std::string const& name, boost::uint32_t position)
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

    if (!compare_no_case(name.c_str(), "Return Number") ||
        !compare_no_case(name.c_str(), "ReturnNumber"))
        return Dimension::Field_ReturnNumber;

    if (!compare_no_case(name.c_str(), "Number of Returns") ||
        !compare_no_case(name.c_str(), "NumberOfReturns"))
        return Dimension::Field_NumberOfReturns;

    if (!compare_no_case(name.c_str(), "Number of Returns"))
        return Dimension::Field_NumberOfReturns;

    if (!compare_no_case(name.c_str(), "Scan Direction") ||
        !compare_no_case(name.c_str(), "ScanDirectionFlag") ||
        !compare_no_case(name.c_str(), "ScanDirection"))
        return Dimension::Field_ScanDirectionFlag;

    if (!compare_no_case(name.c_str(), "Flightline Edge") ||
        !compare_no_case(name.c_str(), "EdgeOfFlightLine") ||
        !compare_no_case(name.c_str(), "FlightlineEdge"))
        return Dimension::Field_EdgeOfFlightLine;

    if (!compare_no_case(name.c_str(), "Classification"))
        return Dimension::Field_Classification;

    if (!compare_no_case(name.c_str(), "Scan Angle Rank") ||
        !compare_no_case(name.c_str(), "ScanAngle") ||
        !compare_no_case(name.c_str(), "ScanAngleRank"))
        return Dimension::Field_ScanAngleRank;

    if (!compare_no_case(name.c_str(), "User Data") ||
        !compare_no_case(name.c_str(), "UserData"))
        return Dimension::Field_UserData;

    if (!compare_no_case(name.c_str(), "Point Source ID")||
        !compare_no_case(name.c_str(), "PointSourceId"))
        return Dimension::Field_PointSourceId;

    if (!compare_no_case(name.c_str(), "Time"))
        return Dimension::Field_Time;

    if (!compare_no_case(name.c_str(), "Red"))
        return Dimension::Field_Red;

    if (!compare_no_case(name.c_str(), "Green"))
        return Dimension::Field_Green;

    if (!compare_no_case(name.c_str(), "Blue"))
        return Dimension::Field_Blue;

    if (!compare_no_case(name.c_str(), "Alpha"))
        return Dimension::Field_Alpha;

    return Dimension::Field_INVALID;
}

Writer::Writer(libpc::Schema const& schema)
 : m_schema(schema) {}

std::string Writer::write()
{
    BufferPtr buffer = BufferPtr(xmlBufferCreate(), BufferDeleter());
    
    xmlBufferPtr b = static_cast<xmlBuffer*>(buffer.get());
    TextWriterPtr writer = TextWriterPtr(xmlNewTextWriterMemory(b, 0), WriterDeleter());

   
    write(writer);

    xmlTextWriterPtr w = static_cast<xmlTextWriterPtr>(writer.get());     
    xmlTextWriterFlush(w);
    // printf("xml: %s", (const char *) b->content);
    return std::string((const char *) b->content, b->size);
    
}

void Writer::write(TextWriterPtr writer)
{

    xmlTextWriterPtr w = static_cast<xmlTextWriterPtr>(writer.get()); 
    
    xmlTextWriterSetIndent(w, 1);
    xmlTextWriterStartDocument(w, NULL, "utf-8", NULL);
    xmlTextWriterStartElementNS(w, BAD_CAST "pc", BAD_CAST "PointCloudSchema", NULL);
    xmlTextWriterWriteAttributeNS(w, BAD_CAST "xmlns", BAD_CAST "pc", NULL, BAD_CAST "http://libpc.org/schemas/PC/1.0");
    xmlTextWriterWriteAttributeNS(w, BAD_CAST "xmlns", BAD_CAST "xsi", NULL, BAD_CAST "http://www.w3.org/2001/XMLSchema-instance");
    
    writeSchema(writer);
    xmlTextWriterEndElement(w);
    xmlTextWriterEndDocument(w);
}

boost::uint32_t GetStreamPrecision(double scale)
{
    double frac = 0;
    double integer = 0;
    
    frac = std::modf(scale, &integer);
    double precision = std::fabs(std::floor(std::log10(frac)));
    
    boost::uint32_t output = static_cast<boost::uint32_t>(precision);
    return output;
}

void Writer::writeSchema(TextWriterPtr writer)
{

    xmlTextWriterPtr w = static_cast<xmlTextWriterPtr>(writer.get()); 
    
    libpc::Schema::Dimensions const& dims = m_schema.getDimensions();
    
    for (boost::uint32_t i = 0; i < dims.size(); i++)
    {
        Dimension const& dim = dims[i];
        xmlTextWriterStartElementNS(w, BAD_CAST "pc", BAD_CAST "dimension", NULL);
        
        std::ostringstream position;
        position << i+1;
        xmlTextWriterWriteElementNS(w, BAD_CAST "pc", BAD_CAST "position", NULL, BAD_CAST position.str().c_str());
        
        std::ostringstream size;
        size << dim.getByteSize();
        xmlTextWriterWriteElementNS(w, BAD_CAST "pc", BAD_CAST "size", NULL, BAD_CAST size.str().c_str());
        
        std::ostringstream description;
        description << dim.getDescription();
        if (description.str().size())
            xmlTextWriterWriteElementNS(w, BAD_CAST "pc", BAD_CAST "description", NULL, BAD_CAST description.str().c_str());
        
        libpc::Dimension::Field f = dim.getField();
        
        std::ostringstream name;
        name << dim.getFieldName(f);
        if (name.str().size())
            xmlTextWriterWriteElementNS(w, BAD_CAST "pc", BAD_CAST "name", NULL, BAD_CAST name.str().c_str());
        
        std::ostringstream type;
        libpc::Dimension::DataType t = dim.getDataType();
    
        switch (t)
        {
            case libpc::Dimension::Int8:
                type << "int8_t";
                break;
            case libpc::Dimension::Uint8:
                type << "uint8_t";
                break;
            case libpc::Dimension::Int16:
                type << "int16_t";
                break;
            case libpc::Dimension::Uint16:
                type << "uint16_t";
                break;
            case libpc::Dimension::Int32:
                type << "int32_t";
                break;
            case libpc::Dimension::Uint32:
                type << "uint32_t";
                break;
            case libpc::Dimension::Int64:
                type << "int64_t";
                break;
            case libpc::Dimension::Uint64:
                type << "uint64_t";
                break;
            case libpc::Dimension::Float:
                type << "float";
                break;
            case libpc::Dimension::Double:
                type << "double";
                break;
            case libpc::Dimension::Undefined:
                type << "unknown";
                break;

   
        }
        xmlTextWriterWriteElementNS(w, BAD_CAST "pc", BAD_CAST "interpretation", NULL, BAD_CAST type.str().c_str());
        
        double minimum = dim.getMinimum();
        if (!Utils::compare_distance<double>(minimum, 0.0))
        {
            std::ostringstream mn;
            mn.setf(std::ios_base::fixed, std::ios_base::floatfield);
            mn.precision(12);
            mn << minimum;
            xmlTextWriterStartElementNS(w, BAD_CAST "pc", BAD_CAST "minimum", NULL);

            xmlTextWriterWriteAttributeNS(w, BAD_CAST "pc", BAD_CAST "units", NULL, BAD_CAST "double");
            xmlTextWriterWriteAttributeNS(w, BAD_CAST "pc", BAD_CAST "value", NULL, BAD_CAST mn.str().c_str());
            xmlTextWriterEndElement(w);            
        }
        

        double maximum = dim.getMaximum();
        if (!Utils::compare_distance<double>(minimum, 0.0))
        {
            std::ostringstream mn;
            mn.setf(std::ios_base::fixed, std::ios_base::floatfield);
            mn.precision(12);
            mn << maximum;
            xmlTextWriterStartElementNS(w, BAD_CAST "pc", BAD_CAST "maximum", NULL);

            xmlTextWriterWriteAttributeNS(w, BAD_CAST "pc", BAD_CAST "units", NULL, BAD_CAST "double");
            xmlTextWriterWriteAttributeNS(w, BAD_CAST "pc", BAD_CAST "value", NULL, BAD_CAST mn.str().c_str());
            xmlTextWriterEndElement(w);            

        }

        double scale = dim.getNumericScale();
        if (!Utils::compare_distance<double>(scale, 0.0))
        {
            std::ostringstream out;
            out.setf(std::ios_base::fixed, std::ios_base::floatfield);
            out.precision(GetStreamPrecision(scale));
            out << scale;
            xmlTextWriterWriteElementNS(w, BAD_CAST "pc", BAD_CAST "scale", NULL, BAD_CAST out.str().c_str());

        }        

        double offset = dim.getNumericOffset();
        if (!Utils::compare_distance<double>(offset, 0.0))
        {
            std::ostringstream out;
            out.setf(std::ios_base::fixed, std::ios_base::floatfield);
            out.precision(12);
            out << offset;
            xmlTextWriterWriteElementNS(w, BAD_CAST "pc", BAD_CAST "offset", NULL, BAD_CAST out.str().c_str());

        }        

        xmlTextWriterWriteElementNS(w, BAD_CAST "pc", BAD_CAST "active", NULL, BAD_CAST "true");

        xmlTextWriterEndElement(w);

        xmlTextWriterFlush(w);
    }
    
}


} } // namespaces
