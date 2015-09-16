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

namespace
{

/**
void print_element_names(xmlNode * a_node)
{

    xmlNode *cur_node = NULL;

    for (cur_node = a_node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            printf("node type: Element, name: %s\n", cur_node->name);
        }
        print_element_names(cur_node->children);
    }
}
**/

} // anonymous namespace

namespace pdal
{

void OCISchemaStructuredErrorHandler
(void * /*userData*/, xmlErrorPtr error)
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
    std::cerr << oss.str() << std::endl;
}

void OCISchemaParserStructuredErrorHandler
(void * /*userData*/, xmlErrorPtr error)
{
    std::cerr << "Schema parsing error: '" << error->message << "' " <<
        "on line " << error->line << std::endl;
}

void OCISchemaValidationStructuredErrorHandler
(void * /*userData*/, xmlErrorPtr error)
{
    std::cerr << "Schema validation error: '" << error->message << "' " <<
        "on line " << error->line << std::endl;
}

void OCISchemaValidityError
(void * /*ctx*/, const char* message, ...)
{
    const int ERROR_MESSAGE_SIZE = 256;
    char error[ERROR_MESSAGE_SIZE];
    va_list arg_ptr;

    va_start(arg_ptr, message);
    vsnprintf(error, ERROR_MESSAGE_SIZE, message, arg_ptr);
    va_end(arg_ptr);

    std::cerr << "Schema validity error: '" << error << "' " << std::endl;
}

void OCISchemaValidityDebug
(void * /*ctx*/, const char* message, ...)
{
    const int ERROR_MESSAGE_SIZE = 256;
    char error[ERROR_MESSAGE_SIZE];
    va_list arg_ptr;

    va_start(arg_ptr, message);
    vsnprintf(error, ERROR_MESSAGE_SIZE, message, arg_ptr);
    va_end(arg_ptr);

    std::cout << "Schema validity debug: '" << error << "' " << "\n";
}


void OCISchemaGenericErrorHandler
(void * /*ctx*/, const char* message, ...)
{
    const int ERROR_MESSAGE_SIZE = 256;
    char error[ERROR_MESSAGE_SIZE];
    va_list arg_ptr;

    va_start(arg_ptr, message);
    vsnprintf(error, ERROR_MESSAGE_SIZE, message, arg_ptr);
    va_end(arg_ptr);

    std::ostringstream oss;

    std::cerr << "Generic error: '" << error << "'" << std::endl;
}

XMLSchema::XMLSchema(std::string xml, std::string xsd,
    Orientation::Enum orientation) : m_orientation(orientation)
{
    xmlDocPtr doc = init(xml, xsd);
    if (doc)
    {
        load(doc);
        xmlFreeDoc(doc);
    }
}


XMLSchema::XMLSchema(const XMLDimList& dims, MetadataNode m,
    Orientation::Enum orientation) : m_orientation(orientation), m_dims(dims),
    m_metadata(m)
{}


XMLSchema::XMLSchema(const PointLayoutPtr& layout, MetadataNode m,
    Orientation::Enum orientation) : m_orientation(orientation), m_metadata(m)
{
    DimTypeList dimTypes = layout->dimTypes();
    for (DimType& d : dimTypes)
        m_dims.push_back(XMLDim(d, layout->dimName(d.m_id)));
}


std::string XMLSchema::xml() const
{
    xmlBuffer *b = xmlBufferCreate();
    xmlTextWriterPtr w = xmlNewTextWriterMemory(b, 0);

    xmlTextWriterSetIndent(w, 1);
    xmlTextWriterStartDocument(w, NULL, "utf-8", NULL);
    xmlTextWriterStartElementNS(w, (const xmlChar*)"pc",
        (const xmlChar*)"PointCloudSchema", NULL);
    xmlTextWriterWriteAttributeNS(w, (const xmlChar*) "xmlns",
        (const xmlChar*)"pc", NULL,
        (const xmlChar*)"http://pointcloud.org/schemas/PC/");
    xmlTextWriterWriteAttributeNS(w, (const xmlChar*)"xmlns",
        (const xmlChar*)"xsi", NULL,
        (const xmlChar*)"http://www.w3.org/2001/XMLSchema-instance");

    writeXml(w);

    xmlTextWriterEndElement(w);
    xmlTextWriterEndDocument(w);

    std::string output((const char *)b->content, b->use);
    xmlFreeTextWriter(w);
    xmlBufferFree(b);

    return output;
}


DimTypeList XMLSchema::dimTypes() const
{
    DimTypeList dimTypes;

    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
        dimTypes.push_back(di->m_dimType);
    return dimTypes;
}


xmlDocPtr XMLSchema::init(const std::string& xml, const std::string& xsd)
{
    xmlParserOption parserOption(XML_PARSE_NONET);

    LIBXML_TEST_VERSION

    xmlSetGenericErrorFunc(m_global_context,
        (xmlGenericErrorFunc)&OCISchemaGenericErrorHandler);
    xmlSetStructuredErrorFunc(m_global_context,
        (xmlStructuredErrorFunc)&OCISchemaStructuredErrorHandler);

    xmlDocPtr doc = xmlReadMemory(xml.c_str(), xml.size(), NULL, NULL,
        parserOption);

    if (xsd.size() && !validate(doc, xsd))
    {
        xmlFreeDoc(doc);
        doc = NULL;
        std::cerr << "Document did not validate against schema." << std::endl;
    }
    return doc;
}


bool XMLSchema::validate(xmlDocPtr doc, const std::string& xsd)
{
    xmlParserOption parserOption(XML_PARSE_NONET);

    xmlDocPtr schemaDoc = xmlReadMemory(xsd.c_str(), xsd.size(),
        NULL, NULL, parserOption);
    xmlSchemaParserCtxtPtr parserCtxt = xmlSchemaNewDocParserCtxt(schemaDoc);
    xmlSchemaSetParserStructuredErrors(parserCtxt,
        &OCISchemaParserStructuredErrorHandler, m_global_context);
    xmlSchemaPtr schema = xmlSchemaParse(parserCtxt);
    xmlSchemaValidCtxtPtr validCtxt = xmlSchemaNewValidCtxt(schema);
    xmlSchemaSetValidErrors(validCtxt, &OCISchemaValidityError,
        &OCISchemaValidityDebug, m_global_context);
    bool valid = (xmlSchemaValidateDoc(validCtxt, doc) == 0);

    xmlFreeDoc(schemaDoc);
    xmlSchemaFreeParserCtxt(parserCtxt);
    xmlSchemaFree(schema);
    xmlSchemaFreeValidCtxt(validCtxt);

    return valid;
}


std::string XMLSchema::remapOldNames(const std::string& input)
{
    if (boost::iequals(input, "Unnamed field 512") ||
            boost::iequals(input, "Chipper Point ID"))
        return std::string("Chipper:PointID");

    if (boost::iequals(input, "Unnamed field 513") ||
            boost::iequals(input, "Chipper Block ID"))
        return std::string("Chipper:BlockID");

    return input;
}


bool XMLSchema::loadMetadata(xmlNode *startNode, MetadataNode& input)
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

    xmlNode *node = startNode;
    for (node = startNode; node; node = node->next)
    {
        if (node->type != XML_ELEMENT_NODE)
            continue;
        if (boost::equals((const char*)node->name, "Metadata"))
        {
            const char *fieldname =
                (const char*)xmlGetProp(node, (const xmlChar*)"name");
            const char *etype =
                (const char*)xmlGetProp(node, (const xmlChar*)"type");
            const char *description =
                (const char*)xmlGetProp(node, (const xmlChar*) "description");
            const char *text = (const char*)xmlNodeGetContent(node);

            if (!boost::iequals(fieldname, "root"))
            {
                if (!fieldname)
                {
                    std::cerr << "Unable to read metadata for node '" <<
                        (const char*)node->name << "' no \"name\" was given";
                    return false;
                }
                input.add(fieldname, text ? text : "",
                    description ? description : "");
            }
        }
        loadMetadata(node->children, input);
    }
    return true;
}


bool XMLSchema::load(xmlDocPtr doc)
{
    xmlNode* root = xmlDocGetRootElement(doc);
    // print_element_names(root);

    if (!boost::iequals((const char*)root->name, "PointCloudSchema"))
    {
        std::cerr << "First node of document was not named 'PointCloudSchema'";
        return false;
    }

    const unsigned SENTINEL_POS = 100000;
    unsigned missingPos = SENTINEL_POS + 1;

    xmlNode* dimension = root->children;
    pdal::Metadata metadata;
    for (xmlNode *dimension = root->children; dimension;
        dimension = dimension->next)
    {
        // Read off orientation setting
        if (boost::equals((const char*)dimension->name, "orientation"))
        {
            xmlChar* n = xmlNodeListGetString(doc, dimension->children, 1);
            if (!n)
            {
                std::cerr << "Unable to fetch orientation.\n";
                return false;
            }
            std::string orientation = std::string((const char*)n);
            xmlFree(n);

            if (boost::iequals(orientation, "dimension"))
                m_orientation = Orientation::DimensionMajor;
            else
                m_orientation = Orientation::PointMajor;
            continue;
        }

        if (boost::equals((const char*)dimension->name, "metadata"))
        {
            m_metadata = MetadataNode("root");
            if (!loadMetadata(dimension, m_metadata))
                return false;
            continue;
        }

        if (dimension->type != XML_ELEMENT_NODE ||
            !boost::iequals((const char*)dimension->name, "dimension"))
            continue;

        XMLDim dim;
        dim.m_position = SENTINEL_POS;
        for (xmlNode *properties = dimension->children; properties;
            properties = properties->next)
        {
            if (properties->type != XML_ELEMENT_NODE)
                continue;

            if (boost::iequals((const char*)properties->name, "name"))
            {
                xmlChar *n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                {
                    std::cerr << "Unable to fetch name from XML node.";
                    return false;
                }
                dim.m_name = remapOldNames(std::string((const char*)n));
                xmlFree(n);
            }
            if (boost::iequals((const char*)properties->name, "description"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                {
                    std::cerr << "Unable to fetch description.\n";
                    return false;
                }
                dim.m_description = std::string((const char*)n);
                xmlFree(n);
            }
            if (boost::iequals((const char*)properties->name, "interpretation"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                {
                    std::cerr << "Unable to fetch interpretation.\n";
                    return false;
                }
                dim.m_dimType.m_type = Dimension::type((const char*)n);
                xmlFree(n);
            }
            if (boost::iequals((const char*)properties->name, "minimum"))
            {
                xmlChar* n = xmlGetProp(properties, (const xmlChar*) "value");
                if (!n)
                {
                    return false;
                    std::cerr << "Unable to fetch minimum value.\n";
                }
                dim.m_min = std::atof((const char*)n);
                xmlFree(n);
            }
            if (boost::iequals((const char*)properties->name, "maximum"))
            {
                xmlChar* n = xmlGetProp(properties, (const xmlChar*) "value");
                if (!n)
                {
                    std::cerr << "Unable to fetch maximum value.\n";
                    return false;
                }
                dim.m_max = std::atof((const char*)n);
                xmlFree(n);
            }
            if (boost::iequals((const char*)properties->name, "position"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                {
                    std::cerr << "Unable to fetch position value.\n";
                    return false;
                }
                dim.m_position = std::atoi((const char*)n);
                xmlFree(n);
            }
            if (boost::iequals((const char*)properties->name, "offset"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                {
                    std::cerr << "Unable to fetch offset value!";
                    return false;
                }
                dim.m_dimType.m_xform.m_offset = std::atof((const char*)n);
                xmlFree(n);
            }
            if (boost::iequals((const char*)properties->name, "scale"))
            {
                xmlChar* n = xmlNodeListGetString(doc, properties->children, 1);
                if (!n)
                {
                    std::cerr << "Unable to fetch scale value!";
                    return false;
                }
                dim.m_dimType.m_xform.m_scale = std::atof((const char*)n);
                xmlFree(n);
            }
        }
        // If we don't have a position, set it to some value larger than all
        // previous values.
        if (dim.m_position == SENTINEL_POS)
            dim.m_position = missingPos++;
        m_dims.push_back(dim);
    }
    std::sort(m_dims.begin(), m_dims.end());

    // Renumber dimension positions to be 1..N
    for (unsigned pos = 0; pos < m_dims.size(); pos++)
        m_dims[pos].m_position = pos + 1;

    return true;
}


XMLDim& XMLSchema::xmlDim(Dimension::Id::Enum id)
{
    static XMLDim nullDim;

    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
        if (di->m_dimType.m_id == id)
            return *di;
    return nullDim;
}


const XMLDim& XMLSchema::xmlDim(Dimension::Id::Enum id) const
{
    static XMLDim nullDim;

    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
        if (di->m_dimType.m_id == id)
            return *di;
    return nullDim;
}


XMLDim& XMLSchema::xmlDim(const std::string& name)
{
    static XMLDim nullDim;

    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
        if (di->m_name == name)
            return *di;
    return nullDim;
}


void XMLSchema::writeXml(xmlTextWriterPtr w) const
{
    int pos = 0;
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di, ++pos)
    {
        xmlTextWriterStartElementNS(w, (const xmlChar*)"pc",
            (const xmlChar*)"dimension", NULL);

        std::ostringstream position;
        position << (pos + 1);
        xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc",
            (const xmlChar*)"position", NULL,
            (const xmlChar*)position.str().c_str());

        std::ostringstream size;
        size << Dimension::size(di->m_dimType.m_type);
        xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc",
            (const xmlChar*)"size", NULL, (const xmlChar*)size.str().c_str());

        std::string description = Dimension::description(di->m_dimType.m_id);
        if (description.size())
            xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc",
                (const xmlChar*)"description", NULL,
                (const xmlChar*)description.c_str());

        XForm xform = di->m_dimType.m_xform;
        if (xform.nonstandard())
        {
            std::ostringstream out;
            out.precision(15);

            out << xform.m_scale;
            std::string scale = out.str();

            out.str(std::string());
            out << xform.m_offset;
            std::string offset = out.str();

            out << xform.m_scale;
            xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc",
                (const xmlChar *)"scale", NULL,
                (const xmlChar *)scale.data());
            xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc",
                (const xmlChar *)"offset", NULL,
                (const xmlChar *)offset.data());
        }

        std::string name = di->m_name;
        if (name.size())
            xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc",
                (const xmlChar*)"name", NULL, (const xmlChar*)name.c_str());

        xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc",
            (const xmlChar*)"interpretation", NULL,
            (const xmlChar*)
                Dimension::interpretationName(di->m_dimType.m_type).c_str());

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
    if (!m_metadata.empty())
    {
        xmlTextWriterStartElementNS(w, (const xmlChar*) "pc",
            (const xmlChar*) "metadata", NULL);

        boost::property_tree::ptree output;
        PipelineWriter::writeMetadata(output, m_metadata.children());
        std::ostringstream oss;
        boost::property_tree::xml_parser::write_xml(oss, output);
        std::string xml = oss.str();

        // wipe off write_xml's xml declaration
        boost::algorithm::erase_all(xml,
            "<?xml version=\"1.0\" encoding=\"utf-8\"?>");
        xmlTextWriterWriteRawLen(w, (const xmlChar*) xml.c_str(), xml.size());
        xmlTextWriterEndElement(w);
    }
    xmlTextWriterWriteElementNS(w, (const xmlChar*) "pc",
        (const xmlChar*)"orientation", NULL,
        (const xmlChar*)orientation.str().c_str());

    xmlTextWriterWriteElementNS(w, (const xmlChar*)"pc", (const xmlChar*)"version", NULL,
                                (const xmlChar*)PDAL_XML_SCHEMA_VERSION);


    xmlTextWriterEndElement(w);
    xmlTextWriterFlush(w);
}

} // namespace pdal
