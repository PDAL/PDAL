/******************************************************************************
* Copyright (c) 2015, Howard Butler (howard@hobu.co)
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

#pragma warning(push)
#pragma warning(disable: 4251)
#include <ogr_geometry.h>
#pragma warning(pop)

#include "Ilvis2MetadataReader.hpp"

namespace pdal
{

void Ilvis2MetadataReader::readMetadataFile(std::string filename,
    MetadataNode* m)
{
    xmlDocPtr doc;
    xmlNodePtr node;

    doc = xmlReadFile(filename.c_str(), NULL, 0);
    if (doc == NULL)
    {
        return;
    }

    node = xmlDocGetRootElement(doc);

    parseGranuleMetaDataFile(node, m);

    xmlFreeDoc(doc);
    xmlCleanupParser();
    xmlMemoryDump();
}


void Ilvis2MetadataReader::parseGranuleMetaDataFile(xmlNodePtr node,
    MetadataNode* m)
{
    assertElementIs(node, "GranuleMetaDataFile");

    xmlNodePtr child = getFirstChildElementNode(node);
    assertElementIs(child, "DTDVersion");
    m->add<double>("DTDVersion", extractDouble(child));

    child = getNextElementNode(child);
    assertElementIs(child, "DataCenterId");
    m->add("DataCenterID", extractString(child));

    child = getNextElementNode(child);
    assertElementIs(child, "GranuleURMetaData");
    parseGranuleURMetaData(child, m);

    child = getNextElementNode(child);
    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parseGranuleURMetaData(xmlNodePtr node,
    MetadataNode* m)
{
    assertElementIs(node, "GranuleURMetaData");

    xmlNodePtr child;

    child = getFirstChildElementNode(node);
    assertElementIs(child, "GranuleUR");
    m->add("GranuleUR", extractString(child));

    child = getNextElementNode(child);
    assertElementIs(child, "DbID");
    m->add<long>("DbID", extractLong(child));

    child = getNextElementNode(child);
    assertElementIs(child, "InsertTime");
    m->add("InsertTime", extractString(child));

    child = getNextElementNode(child);
    assertElementIs(child, "LastUpdate");
    m->add("LastUpdate", extractString(child));

    child = getNextElementNode(child);
    if (nodeElementIs(child, "CollectionMetaData"))
    {
        parseCollectionMetaData(child, m);
        child = getNextElementNode(child);
    }

    if (nodeElementIs(child, "DataFiles"))
    {
        parseDataFiles(child, m);
        child = getNextElementNode(child);
    }

    if (nodeElementIs(child, "ECSDataGranule"))
    {
        parseECSDataGranule(child, m);
        child = getNextElementNode(child);
    }

    if (nodeElementIs(child, "RangeDateTime"))
    {
        parseRangeDateTime(child, m);
        child = getNextElementNode(child);
    }

    if (nodeElementIs(child, "SpatialDomainContainer"))
    {
        parseSpatialDomainContainer(child, m);
        child = getNextElementNode(child);
    }

    while (nodeElementIs(child, "Platform"))
    {
        MetadataNode plat = m->addList("Platform");
        parsePlatform(child, &plat);
        child = getNextElementNode(child);
    }

    while (nodeElementIs(child, "Campaign"))
    {
        parseCampaign(child, m);
        child = getNextElementNode(child);
    }

    if (nodeElementIs(child, "PSAs"))
    {
        parsePSAs(child, m);
        child = getNextElementNode(child);
    }

    if (nodeElementIs(child, "BrowseProduct"))
    {
        parseXXProduct("Browse", child, m);
        child = getNextElementNode(child);
    }

    if (nodeElementIs(child, "PHProduct"))
    {
        parseXXProduct("PH", child, m);
        child = getNextElementNode(child);
    }

    if (nodeElementIs(child, "QAProduct"))
    {
        parseXXProduct("QA", child, m);
        child = getNextElementNode(child);
    }

    if (nodeElementIs(child, "MPProduct"))
    {
        parseXXProduct("MP", child, m);
        child = getNextElementNode(child);
    }

    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parseCollectionMetaData(xmlNodePtr node,
    MetadataNode * m)
{
    assertElementIs(node, "CollectionMetaData");

    xmlNodePtr child = getFirstChildElementNode(node);
    assertElementIs(child, "ShortName");
    m->add("CollectionShortName", extractString(child));

    child = getNextElementNode(child);
    assertElementIs(child, "VersionID");
    m->add("CollectionVersionID", extractInt(child));

    child = getNextElementNode(child);
    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parseDataFiles(xmlNodePtr node, MetadataNode * m)
{
    assertElementIs(node, "DataFiles");

    xmlNodePtr child = getFirstChildElementNode(node);
    assertElementIs(child, "DataFileContainer");

    while(nodeElementIs(child, "DataFileContainer"))
    {
        MetadataNode n = m->addList("DataFile");
        parseDataFileContainer(child, &n);
        child = getNextElementNode(child);
    }

    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parseDataFileContainer(xmlNodePtr node,
    MetadataNode * m)
{
    assertElementIs(node, "DataFileContainer");

    xmlNodePtr child = getFirstChildElementNode(node);
    assertElementIs(child, "DistributedFileName");
    m->add("DistributedFileName", extractString(child));

    child = getNextElementNode(child);
    assertElementIs(child, "FileSize");
    m->add("FileSize", extractInt(child));

    child = getNextElementNode(child);
    if (nodeElementIs(child, "ChecksumType"))
    {
        m->add("ChecksumType", extractString(child));
        child = getNextElementNode(child);
    }

    if (nodeElementIs(child, "Checksum"))
    {
        m->add("Checksum", extractString(child));
        child = getNextElementNode(child);
    }

    if (nodeElementIs(child, "ChecksumOrigin"))
    {
        m->add("ChecksumOrigin", extractString(child));
        child = getNextElementNode(child);
    }

    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parseECSDataGranule(xmlNodePtr node,
    MetadataNode * m)
{
    assertElementIs(node, "ECSDataGranule");

    xmlNodePtr child = getFirstChildElementNode(node);
    if (nodeElementIs(child, "SizeMBECSDataGranule"))
    {
        m->add("SizeMBECSDataGranule", extractDouble(child));
        child = getNextElementNode(child);
    }

    assertElementIs(child, "LocalGranuleID");
    m->add("LocalGranuleID", extractString(child));

    child = getNextElementNode(child);
    if (nodeElementIs(child, "ProductionDateTime"))
    {
        m->add("ProductionDateTime", extractString(child));
        child = getNextElementNode(child);
    }

    assertElementIs(child, "LocalVersionID");
    m->add("LocalVersionID", extractString(child));

    child = getNextElementNode(child);
    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parseRangeDateTime(xmlNodePtr node, MetadataNode * m)
{
    assertElementIs(node, "RangeDateTime");

    xmlNodePtr child = getFirstChildElementNode(node);
    assertElementIs(child, "RangeEndingTime");
    m->add("RangeEndingTime", extractString(child));

    child = getNextElementNode(child);
    assertElementIs(child, "RangeEndingDate");
    m->add("RangeEndingDate", extractString(child));

    child = getNextElementNode(child);
    assertElementIs(child, "RangeBeginningTime");
    m->add("RangeBeginningTime", extractString(child));

    child = getNextElementNode(child);
    assertElementIs(child, "RangeBeginningDate");
    m->add("RangeBeginningDate", extractString(child));

    child = getNextElementNode(child);
    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parseSpatialDomainContainer(xmlNodePtr node,
    MetadataNode * m)
{
    assertElementIs(node, "SpatialDomainContainer");

    xmlNodePtr child = getFirstChildElementNode(node);
    if (nodeElementIs(child, "HorizontalSpatialDomainContainer"))
    {
        xmlNodePtr subChild = getFirstChildElementNode(child);
        assertElementIs(subChild, "GPolygon");
        parseGPolygon(subChild, m);

        child = getNextElementNode(child);
    }

    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parseGPolygon(xmlNodePtr node, MetadataNode * m)
{
    assertElementIs(node, "GPolygon");

    xmlNodePtr child = getFirstChildElementNode(node);
    assertElementIs(child, "Boundary");

    // The number of boundaries is essentially the number of sub-polygons
    int numBoundaries = countChildElements(node, "Boundary");

    // NOTE: Ownership of these rings is transferred to an OGR geometry and
    //   deleted with that geometry.
    std::vector<OGRLinearRing *> rings;
    while (nodeElementIs(child, "Boundary"))
    {
        // There must be at least 3 points to be valid per the schema.
        int numPoints = countChildElements(child, "Point");
        if (numPoints < 3)
            throw error("Found a polygon boundary with less than 3 points, "
                "invalid for this schema");

        xmlNodePtr bdChild = getFirstChildElementNode(child);

        OGRLinearRing *lr = new OGRLinearRing();
        while (nodeElementIs(bdChild, "Point"))
        {
            xmlNodePtr ptChild = getFirstChildElementNode(bdChild);
            assertElementIs(ptChild, "PointLongitude");
            double ptLon = extractDouble(ptChild);

            ptChild = getNextElementNode(ptChild);
            assertElementIs(ptChild, "PointLatitude");
            double ptLat = extractDouble(ptChild);

            ptChild = getNextElementNode(ptChild);
            assertEndOfElements(ptChild);

            lr->addPoint(ptLon, ptLat);

            bdChild = getNextElementNode(bdChild);
        }
        lr->closeRings();
        rings.push_back(lr);
        child = getNextElementNode(child);
    }

    assertEndOfElements(child);

    // If only one sub-polygon, just make a POLYGON WKT,
    // else make it a MULTIPOLYGON
    std::unique_ptr<OGRGeometry> geom;
    if (numBoundaries > 1)
    {
        OGRMultiPolygon *mp = new OGRMultiPolygon();
        for (auto lr : rings)
            mp->addGeometryDirectly(lr);
        geom.reset(mp);
    }
    else
    {
        OGRPolygon *p = new OGRPolygon();
        // Should only be one.
        for (auto lr : rings)
            p->addRingDirectly(lr);
        geom.reset(p);
    }
    char *polyStr;
    geom->exportToWkt(&polyStr);
    m->add("ConvexHull", polyStr);
    CPLFree(polyStr);
}


void Ilvis2MetadataReader::parsePlatform(xmlNodePtr node, MetadataNode * m)
{
    assertElementIs(node, "Platform");

    xmlNodePtr child = getFirstChildElementNode(node);
    assertElementIs(child, "PlatformShortName");

    m->add("PlatformShortName", extractString(child));

    child = getNextElementNode(child);
    while(nodeElementIs(child, "Instrument"))
    {
        MetadataNode inst = m->addList("Instrument");
        parseInstrument(child, &inst);
        child = getNextElementNode(child);
    }

    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parseInstrument(xmlNodePtr node, MetadataNode * m)
{
    assertElementIs(node, "Instrument");

    xmlNodePtr child = getFirstChildElementNode(node);
    assertElementIs(child, "InstrumentShortName");
    m->add("InstrumentShortName", extractString(child));

    child = getNextElementNode(child);

    while (nodeElementIs(child, "Sensor"))
    {
        MetadataNode sens = m->addList("Sensor");
        parseSensor(child, &sens);
        child = getNextElementNode(child);
    }

    while (nodeElementIs(child, "OperationMode"))
    {
        m->addList("OperationMode", extractString(child));
        child = getNextElementNode(child);
    }

    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parseSensor(xmlNodePtr node, MetadataNode * m)
{
    assertElementIs(node, "Sensor");

    xmlNodePtr child = getFirstChildElementNode(node);
    assertElementIs(child, "SensorShortName");
    m->add("SensorShortName", extractString(child));

    child = getNextElementNode(child);
    while (nodeElementIs(child, "SensorCharacteristic"))
    {
        MetadataNode n = m->addList("SensorCharacteristic");
        parseSensorCharacteristic(child, &n);
        child = getNextElementNode(child);
    }

    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parseSensorCharacteristic(xmlNodePtr node,
    MetadataNode * m)
{
    assertElementIs(node, "SensorCharacteristic");

    xmlNodePtr child = getFirstChildElementNode(node);
    assertElementIs(child, "SensorCharacteristicName");
    m->add("CharacteristicName", extractString(child));

    child = getNextElementNode(child);
    assertElementIs(child, "SensorCharacteristicValue");
    m->add("CharacteristicValue", extractString(child));

    child = getNextElementNode(child);
    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parseCampaign(xmlNodePtr node, MetadataNode * m)
{
    assertElementIs(node, "Campaign");

    xmlNodePtr child = getFirstChildElementNode(node);
    assertElementIs(child, "CampaignShortName");
    std::string cName = extractString(child);
    m->addList("Campaign", cName);

    child = getNextElementNode(child);
    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parsePSAs(xmlNodePtr node, MetadataNode * m)
{
    assertElementIs(node, "PSAs");

    xmlNodePtr child = getFirstChildElementNode(node);
    while (nodeElementIs(child, "PSA"))
    {
        MetadataNode n = m->addList("PSA");
        parsePSA(child, &n);
        child = getNextElementNode(child);
    }

    assertEndOfElements(child);
}


void Ilvis2MetadataReader::parsePSA(xmlNodePtr node, MetadataNode * m)
{
    assertElementIs(node, "PSA");

    xmlNodePtr child = getFirstChildElementNode(node);
    assertElementIs(child, "PSAName");
    m->add("PSAName", extractString(child));

    child = getNextElementNode(child);
    while (nodeElementIs(child, "PSAValue"))
    {
        m->addList("PSAValue", extractString(child));
        child = getNextElementNode(child);
    }

    assertEndOfElements(child);
}


// Since the Browse, PH, QA, and MP product nodes have the same structure
// just differing prefixes, they can share this code.
void Ilvis2MetadataReader::parseXXProduct(std::string type, xmlNodePtr node,
    MetadataNode * m)
{
    std::string fullBase = type + "Product";
    std::string fullSub = type + "GranuleId";
    std::string mdName = type + "ProductGranuleId";

    assertElementIs(node, fullBase);

    xmlNodePtr child = getFirstChildElementNode(node);
    while (nodeElementIs(child, fullSub))
    {
        m->addList(mdName, extractString(child));
        child = getNextElementNode(child);
    }

    assertEndOfElements(child);
}


// BEGIN PRIVATE METHODS


std::string Ilvis2MetadataReader::extractString(xmlNodePtr node)
{
    std::string nodeStr((char*)node->children->content);
    return nodeStr;
}


double Ilvis2MetadataReader::extractDouble(xmlNodePtr node)
{
    return atof((char*)node->children->content);
}


int Ilvis2MetadataReader::extractInt(xmlNodePtr node)
{
    return atoi((char*)node->children->content);
}


long Ilvis2MetadataReader::extractLong(xmlNodePtr node)
{
    return atol((char*)node->children->content);
}


// private

// Skip all non-element nodes, just get the next element node.
xmlNodePtr Ilvis2MetadataReader::getNextElementNode(xmlNodePtr node)
{
    node = node->next;
    while (node && node->type != XML_ELEMENT_NODE)
    {
        node = node->next;
    }

    return node;
}

// Skip all non-element child nodes, get the first element child node
xmlNodePtr Ilvis2MetadataReader::getFirstChildElementNode(xmlNodePtr node)
{
    xmlNodePtr child = node->children;
    if (!child)
    {
        return NULL;
    }
    else if (child->type == XML_ELEMENT_NODE)
    {
        return child;
    }
    else
    {
        return getNextElementNode(child);
    }
}

// Verifies the name of the node matches what's expected
bool Ilvis2MetadataReader::nodeElementIs(xmlNodePtr node, std::string expected)
{
    if (!node)
    {
        return false;
    }

    return xmlStrcmp(node->name,
            reinterpret_cast<const xmlChar*>(expected.c_str())) == 0;
}


// Throws an error if the next element is not what it expects
void Ilvis2MetadataReader::assertElementIs(xmlNodePtr node, std::string expected)
{
    if (!node || !nodeElementIs(node, expected))
        throw error("Expected element '" + expected + "', found '" +
            std::string((const char *)node->name) + "'");
}


// Throws an error if the node is not null
void Ilvis2MetadataReader::assertEndOfElements(xmlNodePtr node)
{
    if (node)
        throw("Expected to find no more elements, found '" +
            std::string((const char *)node->name) + "'");
}


// Counts the number of child element nodes with a given name
int Ilvis2MetadataReader::countChildElements(xmlNodePtr node,
    std::string childName)
{
    xmlNodePtr child = getFirstChildElementNode(node);
    int ctr = 0;

    while (child)
    {
        if (nodeElementIs(child, childName))
        {
            ctr += 1;
        }
        child = getNextElementNode(child);
    }

    return ctr;
}

} // namespace pdal

