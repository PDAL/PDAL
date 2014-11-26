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
#include <pdal/PDALUtils.hpp>

#include <sstream>
#include <iostream>
#include <iostream>
#include <list>
#include <cstdlib>
#include <map>
#include <algorithm>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

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
    if (m_xml.empty())
        throw schema_generic_error("Inputted XML is empty, "
            "is there data there?");

#ifdef PDAL_HAVE_LIBXML2
    LIBXML_TEST_VERSION

    xmlSetGenericErrorFunc(m_global_context,
        (xmlGenericErrorFunc)&OCISchemaGenericErrorHandler);
    xmlSetStructuredErrorFunc(m_global_context,
        (xmlStructuredErrorFunc)&OCISchemaStructuredErrorHandler);

    m_doc = DocPtr(xmlReadMemory(m_xml.c_str(), m_xml.size(), NULL, NULL,
        m_doc_options), XMLDocDeleter());

    if (m_xsd.size())
    {
        m_schema_doc = DocPtr(xmlReadMemory(m_xsd.c_str(), m_xsd.size(),
            NULL, NULL, m_doc_options), XMLDocDeleter());

        m_schema_parser_ctx = SchemaParserCtxtPtr(
            xmlSchemaNewDocParserCtxt(static_cast<xmlDocPtr>(
                m_schema_doc.get())), SchemaParserCtxDeleter());

        xmlSchemaSetParserStructuredErrors(
            static_cast<xmlSchemaParserCtxtPtr>(m_schema_parser_ctx.get()),
                &OCISchemaParserStructuredErrorHandler, m_global_context);


        m_schema_ptr = SchemaPtr(
            xmlSchemaParse(static_cast<xmlSchemaParserCtxtPtr>(
                m_schema_parser_ctx.get())), SchemaDeleter());

        m_schema_valid_ctx = SchemaValidCtxtPtr(
            xmlSchemaNewValidCtxt(static_cast<xmlSchemaPtr>(
                m_schema_ptr.get())), SchemaValidCtxtDeleter());

        xmlSchemaSetValidErrors(
            static_cast<xmlSchemaValidCtxtPtr>(m_schema_valid_ctx.get()),
                &OCISchemaValidityError, &OCISchemaValidityDebug,
                m_global_context);

        int valid_schema = xmlSchemaValidateDoc(
            static_cast<xmlSchemaValidCtxtPtr>(m_schema_valid_ctx.get()),
            static_cast<xmlDocPtr>(m_doc.get()));

        if (valid_schema != 0)
            throw schema_error("Document did not validate against schema!");

    }
#endif
}


Reader::Reader(std::string xml, std::string xsd)
    : m_doc_options(XML_PARSE_NONET), m_field_position(512)
{
    m_xml = xml;
    m_xsd = xsd;
    Initialize();
    Load();
}


Reader::~Reader()
{
#ifdef PDAL_HAVE_LIBXML2
    xmlCleanupParser();
#endif
}


// static void
// print_element_names(xmlNode * a_node)
// {
// #ifdef PDAL_HAVE_LIBXML2
// //
//     xmlNode *cur_node = NULL;
// //
//     for (cur_node = a_node; cur_node; cur_node = cur_node->next)
//     {
//         if (cur_node->type == XML_ELEMENT_NODE)
//         {
//             printf("node type: Element, name: %s\n", cur_node->name);
//         }
// //
//         print_element_names(cur_node->children);
//     }
// #endif
// }
//
std::string Reader::remapOldNames(std::string const& input)
{
    if (boost::iequals(input, "Unnamed field 512") || boost::iequals(input, "Chipper Point ID"))
        return std::string("Chipper:PointID");

    if (boost::iequals(input, "Unnamed field 513") || boost::iequals(input, "Chipper Block ID"))
        return std::string("Chipper:BlockID");

    return input;
}

#ifdef PDAL_HAVE_LIBXML2
MetadataNode Reader::LoadMetadata(xmlNode* startNode, MetadataNode& input)
{
//     Expect metadata in the following form
//     We are going to skip the root element because we are
//     expecting to be given one with our input
//     <pc:metadata>
//         <Metadata name="root" type="">
//             <Metadata name="compression" type="string">lazperf</Metadata>
//             <Metadata name="version" type="string">1.0</Metadata>
//         </Metadata>
//     </pc:metadata>


    xmlNode* node = startNode;
    for (node = startNode; node; node=node->next)
    {
        if (node->type == XML_ELEMENT_NODE)
        {
            if (boost::equals((const char*)node->name, "Metadata"))
            {
                const char* fieldname = (const char*) xmlGetProp(node, (const xmlChar*) "name");
                const char* etype = (const char*) xmlGetProp(node, (const xmlChar*) "type");
                const char* description = (const char*)  xmlGetProp(node, (const xmlChar*) "description");
                const char* text = (const char*) xmlNodeGetContent(node);

                if (!boost::iequals(fieldname, "root"))
                {
                    if (!fieldname)
                    {
                        std::ostringstream oss;
                        oss << "Unable to read metadata for node '" << (const char*)node->name<<"' no \"name\" was given";
                        throw pdal_error(oss.str());
                    }
                    input.add(fieldname, text ? text : "", description ? description : "");
                }
            }
            LoadMetadata(node->children, input);
        }
    }

    return input;
}
#endif


void Reader::Load()
{
#ifdef PDAL_HAVE_LIBXML2

    xmlDocPtr doc = static_cast<xmlDocPtr>(m_doc.get());
    xmlNode* root = xmlDocGetRootElement(doc);
    // print_element_names(root);

    if (!boost::iequals((const char*)root->name, "PointCloudSchema"))
        throw schema_loading_error("First node of document was not "
            "named 'PointCloudSchema'");

    xmlNode* dimension = root->children;

    pdal::Metadata metadata;

    while (dimension)
    {
        // Read off orientation setting
        if (boost::equals((const char*)dimension->name, "orientation"))
        {
            xmlChar* n = xmlNodeListGetString(doc, dimension->children, 1);
            if (!n)
                throw schema_loading_error("Unable to fetch orientation!");
            std::string orientation = std::string((const char*)n);
            xmlFree(n);

            if (boost::iequals(orientation, "dimension"))
                m_schema.m_orientation = Orientation::DimensionMajor;
            else
                m_schema.m_orientation = Orientation::PointMajor;

            dimension = dimension->next;
            continue;
        }

        if (boost::equals((const char*)dimension->name, "metadata"))
        {
            m_metadata = MetadataNode("root");
            m_metadata = LoadMetadata(dimension, m_metadata);

            dimension = dimension->next;
            continue;
        }

        if (dimension->type != XML_ELEMENT_NODE ||
            !boost::iequals((const char*)dimension->name, "dimension"))
        {
            dimension = dimension->next;
            continue;
        }

        xmlNode* properties = dimension->children;

        DimInfo info;

        while (properties != NULL)
        {
            if (properties->type != XML_ELEMENT_NODE)
            {
                properties = properties->next;
                continue;
            }

            if (boost::iequals((const char*)properties->name, "name"))
            {
                CharPtr n = CharPtr(xmlNodeListGetString(doc,
                    properties->children, 1), xmlCharDeleter());
                if (!n)
                    throw schema_loading_error("Unable to fetch name!");
                info.m_name = remapOldNames(std::string((const char*)n.get()));
            }

            /**
            //We don't care about size, since size is embedded in the type.
            if (boost::iequals((const char*)properties->name, "size"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                    throw schema_loading_error("Unable to fetch size!");
                int s = std::atoi((const char*)n);
                if (s < 1)
                    throw schema_loading_error("Dimension size is < 1!");
                xmlFree(n);
                size = static_cast<boost::uint32_t>(s);
            }
            **/

            if (boost::iequals((const char*)properties->name, "position"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                    throw schema_loading_error("Unable to fetch position!");
                int p = std::atoi((const char*)n);
                if (p < 1)
                    throw schema_loading_error("Dimension position is < 1!");
                xmlFree(n);
                info.m_position = static_cast<boost::uint32_t>(p);
            }
            if (boost::iequals((const char*)properties->name, "description"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                    throw schema_loading_error("Unable to fetch description!");
                info.m_description = std::string((const char*)n);
                xmlFree(n);
            }
            if (boost::iequals((const char*)properties->name, "interpretation"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                    throw schema_loading_error("Unable to fetch "
                        "interpretation!");
                info.m_type = Dimension::type((const char*)n);
                xmlFree(n);
            }

            if (boost::iequals((const char*)properties->name, "minimum"))
            {
                xmlChar* n = xmlGetProp(properties, (const xmlChar*) "value");
                if (!n)
                    throw schema_loading_error("Unable to fetch "
                        "minimum value!");

                info.m_min = std::atof((const char*)n);
                xmlFree(n);
            }

            if (boost::iequals((const char*)properties->name, "maximum"))
            {
                xmlChar* n = xmlGetProp(properties, (const xmlChar*) "value");
                if (!n)
                    throw schema_loading_error("Unable to fetch maximum "
                        "value!");

                info.m_max = std::atof((const char*)n);
                xmlFree(n);
            }

            if (boost::iequals((const char*)properties->name, "offset"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                    throw schema_loading_error("Unable to fetch offset value!");

                info.m_offset = std::atof((const char*)n);
                xmlFree(n);
            }
            if (boost::iequals((const char*)properties->name, "scale"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                    throw schema_loading_error("Unable to fetch scale value!");

                info.m_scale = std::atof((const char*)n);
                xmlFree(n);
            }

            /**
            if (boost::iequals((const char*)properties->name, "uuid"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                    throw schema_loading_error("Unable to fetch uuid value!");
                uuid = std::string((const char*)n);

                xmlFree(n);
            }

            if (boost::iequals((const char*)properties->name, "parent_uuid"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                    throw schema_loading_error("Unable to fetch uuid value!");
                parent_uuid = std::string((const char*)n);

                xmlFree(n);
            }
            **/
            properties = properties->next;
        }

        m_schema.m_dims.push_back(info);
        dimension = dimension->next;
    }

    std::sort(m_schema.m_dims.begin(), m_schema.m_dims.end());
#endif
}


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
    return std::string((const char *) b->content, b->use);
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
    xmlTextWriterStartElementNS(w, (const xmlChar*) "pc", (const xmlChar*) "PointCloudSchema", NULL);
    xmlTextWriterWriteAttributeNS(w, (const xmlChar*) "xmlns", (const xmlChar*) "pc", NULL, (const xmlChar*) "http://pointcloud.org/schemas/PC/");
    xmlTextWriterWriteAttributeNS(w, (const xmlChar*) "xmlns", (const xmlChar*) "xsi", NULL, (const xmlChar*) "http://www.w3.org/2001/XMLSchema-instance");

    writeSchema(writer);

    xmlTextWriterEndElement(w);
    xmlTextWriterEndDocument(w);
    xmlTextWriterFlush(w);
#endif
}


void Writer::writeSchema(TextWriterPtr writer)
{
#ifdef PDAL_HAVE_LIBXML2

    xmlTextWriterPtr w = static_cast<xmlTextWriterPtr>(writer.get());

    int pos = 0;
    for (auto di = m_dimTypes.begin(); di != m_dimTypes.end(); ++di, ++pos)
    {
        xmlTextWriterStartElementNS(w, (const xmlChar*)"pc",
            (const xmlChar*)"dimension", NULL);

        std::ostringstream position;
        position << (pos + 1);
        xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc",
            (const xmlChar*)"position", NULL,
            (const xmlChar*)position.str().c_str());

        std::ostringstream size;
        size << Dimension::size(di->m_type);
        xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc",
            (const xmlChar*)"size", NULL, (const xmlChar*)size.str().c_str());

        std::string description = Dimension::description(di->m_id);
        if (description.size())
            xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc",
                (const xmlChar*)"description", NULL,
                (const xmlChar*)description.c_str());

        std::string name = Dimension::name(di->m_id);
        if (name.size())
            xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc",
                (const xmlChar*)"name", NULL, (const xmlChar*)name.c_str());

        xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc",
            (const xmlChar*)"interpretation", NULL,
            (const xmlChar*) Dimension::interpretationName(di->m_type).c_str());

        xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc",
            (const xmlChar*)"active", NULL, (const xmlChar*)"true");

        xmlTextWriterEndElement(w);
        xmlTextWriterFlush(w);
    }
    std::ostringstream orientation;
    if (m_orientation == Orientation::PointMajor)
        orientation << "point";
    if (m_orientation == Orientation::DimensionMajor)
        orientation << "dimension";
    xmlTextWriterWriteElementNS(w, (const xmlChar*) "pc",
        (const xmlChar*)"orientation", NULL,
        (const xmlChar*)orientation.str().c_str());

    if (!m_metadata.empty())
    {
        xmlTextWriterStartElementNS(w, (const xmlChar*) "pc",
            (const xmlChar*) "metadata", NULL);

        boost::property_tree::ptree output;
        PipelineWriter::writeMetadata(output, m_metadata);
        std::ostringstream oss;
        boost::property_tree::xml_parser::write_xml(oss, output);
        std::string xml = oss.str();

        // wipe off write_xml's xml declaration
        boost::algorithm::erase_all(xml, "<?xml version=\"1.0\" encoding=\"utf-8\"?>");
        xmlTextWriterWriteRawLen(w, (const xmlChar*) xml.c_str(), xml.size());
        xmlTextWriterEndElement(w);
    }

    xmlTextWriterEndElement(w);
    xmlTextWriterFlush(w);

#endif
}

} // namespace schema
} // namespace pdal
