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

#include <pdal/XMLSchema.hpp>
#include <pdal/PipelineWriter.hpp>

#include <sstream>
#include <iostream>
#include <list>
#include <cstdlib>
#include <map>
#include <algorithm>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <string.h>
#include <stdlib.h>

#ifdef PDAL_HAVE_LIBXML2

struct XMLDocDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
            ::xmlFreeDoc(ptr);
    }
};

struct SchemaParserCtxDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
            ::xmlSchemaFreeParserCtxt(ptr);
    }
};

struct SchemaDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
            ::xmlSchemaFree(ptr);
    }
};

struct SchemaValidCtxtDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
            ::xmlSchemaFreeValidCtxt(ptr);
    }
};

struct WriterDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
            ::xmlFreeTextWriter(ptr);
    }
};

struct BufferDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
            ::xmlBufferFree(ptr);
    }
};

struct xmlCharDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
            ::xmlFree(ptr);
    }
};

#endif

static bool sort_dimensions(pdal::Dimension const& a, pdal::Dimension const& b)
{
    return a < b;
}

namespace pdal
{
namespace schema
{

#ifdef PDAL_HAVE_LIBXML2

void OCISchemaStructuredErrorHandler
(void * userData, xmlErrorPtr error)
{
    boost::ignore_unused_variable_warning(userData);

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
    boost::ignore_unused_variable_warning(userData);

    std::ostringstream oss;

    oss << "Schema parsing error: '" << error->message <<"' ";
    oss << "on line " << error->line;
    throw schema_parsing_error(oss.str());
}

void OCISchemaValidationStructuredErrorHandler
(void * userData, xmlErrorPtr error)
{
    boost::ignore_unused_variable_warning(userData);

    std::ostringstream oss;

    oss << "Schema validation error: '" << error->message <<"' ";
    oss << "on line " << error->line;
    throw schema_validation_error(oss.str());
}

void OCISchemaValidityError
(void * ctx, const char* message, ...)
{
    boost::ignore_unused_variable_warning(ctx);

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
(void * ctx, const char* message, ...)
{
    boost::ignore_unused_variable_warning(ctx);

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
(void * ctx, const char* message, ...)
{
    boost::ignore_unused_variable_warning(ctx);

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
#endif



// XML_PARSE_NONET No network access
// http://xmlsoft.org/html/libxml-parser.html#xmlParserOption


void Reader::Initialize()
{
    if (m_xml.size() == 0) throw schema_generic_error("Inputted XML has no size, is there data there?");

    // if (m_xsd.size() == 0) throw schema_generic_error("Inputted XSD has no size, is there data there?");

#ifdef PDAL_HAVE_LIBXML2


    LIBXML_TEST_VERSION

    xmlSetGenericErrorFunc(m_global_context, (xmlGenericErrorFunc) &OCISchemaGenericErrorHandler);
    xmlSetStructuredErrorFunc(m_global_context, (xmlStructuredErrorFunc) & OCISchemaStructuredErrorHandler);


    m_doc = DocPtr(
                xmlReadMemory(m_xml.c_str(), m_xml.size(), NULL, NULL, m_doc_options),
                XMLDocDeleter());

    if (m_xsd.size())
    {
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
#endif


}

Reader::Reader(std::string const& xml, std::string const &xsd)
    : m_doc_options(XML_PARSE_NONET)
    , m_field_position(512)
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
        throw schema_generic_error("pdal::schema::Reader: xml istream pointer was null!");

    std::istream::pos_type size;

    std::vector<char> data;
    xml->seekg(0, std::ios::end);
    size = xml->tellg();
    data.resize(static_cast<std::vector<char>::size_type>(size));
    xml->seekg(0, std::ios::beg);
    xml->read(&data.front(), size);
    xml->seekg(0, std::ios::beg);

    m_xml = std::string(&data[0], data.size());

    if (xsd)
    {

        std::istream::pos_type size;

        std::vector<char> data;
        xsd->seekg(0, std::ios::end);
        size = xsd->tellg();
        data.resize(static_cast<std::vector<char>::size_type>(size));
        xsd->seekg(0, std::ios::beg);
        xsd->read(&data.front(), size);
        xsd->seekg(0, std::ios::end);

        m_xsd = std::string(&data[0], data.size());
    }

    Initialize();
    Load();
}

Reader::~Reader()
{
#ifdef PDAL_HAVE_LIBXML2
    xmlCleanupParser();
#endif
}


static void
print_element_names(xmlNode * a_node)
{
#ifdef PDAL_HAVE_LIBXML2

    xmlNode *cur_node = NULL;

    for (cur_node = a_node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            printf("node type: Element, name: %s\n", cur_node->name);
        }

        print_element_names(cur_node->children);
    }
#endif
}

std::string Reader::remapOldNames(std::string const& input)
{
    if (boost::iequals(input, "Unnamed field 512") || boost::iequals(input, "Chipper Point ID"))
        return std::string("Chipper:PointID");

    if (boost::iequals(input, "Unnamed field 513") || boost::iequals(input, "Chipper Block ID"))
        return std::string("Chipper:BlockID");

    return input;
}

#ifdef PDAL_HAVE_LIBXML2
pdal::Metadata Reader::LoadMetadata(xmlNode* startNode)
{

    pdal::Metadata output;


    xmlNode* node = startNode;


//     xmlChar* name = xmlGetProp(node, (const xmlChar*) "name");
//     xmlChar* etype = xmlGetProp(node, (const xmlChar*) "type");
// print_element_names(node);
    // std::cout << "node name: " << (const char*)node->name << std::endl;
//         std::cout << "prop type: " << (const char*) etype << std::endl;

    // pdal::Metadata m((const char*) node->name);
    // if (boost::iequals((const char*)etype, "blank"))
    // {
    //     // blank denotes a new Metadata instance.
    //     if (node->children)
    //         output.addMetadata(LoadMetadata(node->children));
    // }

    //

    while (node != NULL)
    {

        //     std::cout << "node name: " << (const char*)node->name << std::endl;

        if (node->properties)
        {
//           xmlChar* name = xmlGetProp(node, (const xmlChar *)"name");
//           xmlChar* etype = xmlGetProp(node, (const xmlChar *)"type");
//            std::cout << "property name: " << (const char*)name << std::endl;
            // std::cout << "proper type: " << (const char*)etype << std::endl;

        }

        // pdal::Metadata m((const char*) node->name);
        // if (boost::iequals((const char*)etype.get(), "blank"))
        // {
        //     // blank denotes a new Metadata instance.
        //     m.addMetadata(LoadMetadata(node));
        // }


        // output.addMetadata(m);

        if (node->type == XML_ELEMENT_NODE)
        {
            node = node->children;
        }
        else
            node = node->next;
    }
    return output;
}
#endif

void Reader::Load()
{
    std::vector<pdal::Dimension> layouts;

#ifdef PDAL_HAVE_LIBXML2

    xmlDocPtr doc = static_cast<xmlDocPtr>(m_doc.get());
    xmlNode* root = xmlDocGetRootElement(doc);
    // print_element_names(root);


    if (!boost::iequals((const char*)root->name, "PointCloudSchema"))
        throw schema_loading_error("First node of document was not named 'PointCloudSchema'");

    xmlNode* dimension = root->children;

    pdal::Metadata metadata;

    while (dimension != NULL)
    {
        // printf("node name: %s\n", (const char*)dimension->name);
        // if (boost::equals((const char*)dimension->name, "metadata"))
        // {
        //     printf("metadata node name: %s\n", (const char*)dimension->name);
        //
        //
        //     metadata.addMetadata(LoadMetadata(dimension));
        //     dimension = dimension->next;
        //     continue;
        // }

        if (dimension->type != XML_ELEMENT_NODE || !boost::iequals((const char*)dimension->name, "dimension"))
        {
            dimension = dimension->next;
            continue;
        }





        xmlNode* properties = dimension->children;

        std::string name;
        boost::uint32_t size(0);
        boost::uint32_t position(1);
        std::string description;
        std::string interpretation;
        std::string uuid;
        std::string parent_uuid;
        double offset(0.0);
        double scale(0.0);
        double minimum(0.0);
        double maximum(0.0);
        EndianType endianness = Endian_Little;

        while (properties != NULL)
        {
            if (properties->type != XML_ELEMENT_NODE)
            {
                properties = properties->next;
                continue;
            }

            if (boost::iequals((const char*)properties->name, "name"))
            {
                CharPtr n = CharPtr(
                                xmlNodeListGetString(doc, properties->children, 1),
                                xmlCharDeleter());
                // xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch name!");
                name = std::string((const char*)n.get());
                name = remapOldNames(name);
                // xmlFree(n);
                // std::cout << "Dimension name: " << name << std::endl;
            }

            if (boost::iequals((const char*)properties->name, "size"))
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

            if (boost::iequals((const char*)properties->name, "position"))
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
            if (boost::iequals((const char*)properties->name, "description"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch description!");
                description = std::string((const char*)n);
                xmlFree(n);
            }
            if (boost::iequals((const char*)properties->name, "interpretation"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch interpretation!");
                interpretation = std::string((const char*)n);
                xmlFree(n);
            }

            if (boost::iequals((const char*)properties->name, "minimum"))
            {
                xmlChar* n = xmlGetProp(properties, (const xmlChar*) "value");
                if (!n) throw schema_loading_error("Unable to fetch minimum value!");

                minimum = std::atof((const char*)n);
                xmlFree(n);
                // std::cout << "Dimension minimum: " << minimum << std::endl;
            }

            if (boost::iequals((const char*)properties->name, "maximum"))
            {
                xmlChar* n = xmlGetProp(properties, (const xmlChar*) "value");
                if (!n) throw schema_loading_error("Unable to fetch maximum value!");

                maximum = std::atof((const char*)n);
                xmlFree(n);
                // std::cout << "Dimension maximum: " << maximum << std::endl;
            }

            if (boost::iequals((const char*)properties->name, "offset"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch offset value!");

                offset = std::atof((const char*)n);
                xmlFree(n);
                // std::cout << "Dimension offset: " << offset << std::endl;
            }
            if (boost::iequals((const char*)properties->name, "scale"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch scale value!");

                scale = std::atof((const char*)n);
                xmlFree(n);
                // std::cout << "Dimension scale: " << scale << std::endl;
            }
            if (boost::iequals((const char*)properties->name, "endianness"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch endianness value!");

                if (boost::iequals((const char*) n, "big"))
                    endianness = Endian_Big;
                else
                    endianness = Endian_Little;

                xmlFree(n);
                // std::cout << "Dimension endianness: " << endianness << std::endl;
            }
            if (boost::iequals((const char*)properties->name, "uuid"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch uuid value!");
                uuid = std::string((const char*)n);

                xmlFree(n);
            }

            if (boost::iequals((const char*)properties->name, "parent_uuid"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n) throw schema_loading_error("Unable to fetch uuid value!");
                parent_uuid = std::string((const char*)n);

                xmlFree(n);
            }

            // printf("property name: %s\n", properties->name);
            properties = properties->next;
        }

        dimension::Interpretation interp = Dimension::getInterpretation(interpretation);

        Dimension d(name, interp, size, description);
        if (uuid.size())
            d.setUUID(uuid);
        if (parent_uuid.size())
        {
            boost::uuids::string_generator gen;
            d.setParent(gen(parent_uuid));
        }
        if (! Utils::compare_distance(scale, 0.0))
        {
            d.setNumericScale(scale);
        }
        if (! Utils::compare_distance(offset, 0.0))
        {
            d.setNumericOffset(offset);
        }
        if (! Utils::compare_distance(minimum, 0.0))
        {
            d.setMinimum(minimum);
        }
        if (! Utils::compare_distance(maximum, 0.0))
        {
            d.setMaximum(maximum);
        }

        d.setEndianness(endianness);

        d.setPosition(position);

        if (d.getUUID() == boost::uuids::nil_uuid())
        {
            d.createUUID();
        }
        layouts.push_back(d);

        dimension = dimension->next;
    }

    std::sort(layouts.begin(), layouts.end(), sort_dimensions);

    std::vector<Dimension>::const_iterator i;
    for (i = layouts.begin(); i!= layouts.end(); ++i)
    {
        const Dimension& dim = *i;
        m_schema.appendDimension(dim);
    }
#endif

}




Writer::Writer(pdal::Schema const& schema)
    : m_schema(schema) {}

std::string Writer::getXML()
{
#ifdef PDAL_HAVE_LIBXML2

    BufferPtr buffer = BufferPtr(xmlBufferCreate(), BufferDeleter());

    xmlBufferPtr b = static_cast<xmlBuffer*>(buffer.get());
    TextWriterPtr writer = TextWriterPtr(xmlNewTextWriterMemory(b, 0), WriterDeleter());

    write(writer);

    xmlTextWriterPtr w = static_cast<xmlTextWriterPtr>(writer.get());
    xmlTextWriterFlush(w);
    // printf("xml: %s", (const char *) b->content);
    return std::string((const char *) b->content, b->size);
#else
    return std::string();
#endif

}

void Writer::write(TextWriterPtr writer)
{
#ifdef PDAL_HAVE_LIBXML2

    xmlTextWriterPtr w = static_cast<xmlTextWriterPtr>(writer.get());

    xmlTextWriterSetIndent(w, 1);
    xmlTextWriterStartDocument(w, NULL, "utf-8", NULL);
    xmlTextWriterStartElementNS(w, BAD_CAST "pc", BAD_CAST "PointCloudSchema", NULL);
    xmlTextWriterWriteAttributeNS(w, BAD_CAST "xmlns", BAD_CAST "pc", NULL, BAD_CAST "http://pointcloud.org/schemas/PC/1.1");
    xmlTextWriterWriteAttributeNS(w, BAD_CAST "xmlns", BAD_CAST "xsi", NULL, BAD_CAST "http://www.w3.org/2001/XMLSchema-instance");

    writeSchema(writer);

    if (m_metadata.size())
    {
        xmlTextWriterStartElementNS(w, BAD_CAST "pc", BAD_CAST "metadata", NULL);

        boost::property_tree::ptree output;
        PipelineWriter::write_metadata_ptree(output, m_metadata);
        std::ostringstream oss;
        boost::property_tree::xml_parser::write_xml(oss, output);
        std::string xml = oss.str();

        // wipe off write_xml's xml declaration
        boost::algorithm::erase_all(xml, "<?xml version=\"1.0\" encoding=\"utf-8\"?>");
        xmlTextWriterWriteRawLen(w, BAD_CAST xml.c_str(), xml.size());
        xmlTextWriterEndElement(w);
    }


    xmlTextWriterEndElement(w);
    xmlTextWriterEndDocument(w);
#endif
}


void Writer::writeSchema(TextWriterPtr writer)
{
#ifdef PDAL_HAVE_LIBXML2

    xmlTextWriterPtr w = static_cast<xmlTextWriterPtr>(writer.get());

    schema::index_by_index const& dims = m_schema.getDimensions().get<schema::index>();

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

        std::ostringstream name;
        name << dim.getName();
        if (name.str().size())
            xmlTextWriterWriteElementNS(w, BAD_CAST "pc", BAD_CAST "name", NULL, BAD_CAST name.str().c_str());

        xmlTextWriterWriteElementNS(w, BAD_CAST "pc", BAD_CAST "interpretation", NULL, BAD_CAST dim.getInterpretationName().c_str());

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
            out.precision(14);
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

        std::ostringstream uuid;
        uuid << dim.getUUID();
        if (uuid.str().size())
            xmlTextWriterWriteElementNS(w, BAD_CAST "pc", BAD_CAST "uuid", NULL, BAD_CAST uuid.str().c_str());

        std::ostringstream parent;
        parent << dim.getParent();
        if (parent.str().size())
            xmlTextWriterWriteElementNS(w, BAD_CAST "pc", BAD_CAST "parent_uuid", NULL, BAD_CAST parent.str().c_str());

        xmlTextWriterEndElement(w);

        xmlTextWriterFlush(w);
    }
#endif

}


}
} // namespaces
