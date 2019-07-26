/******************************************************************************
* Copyright (c) 2019, Helix Re Inc.
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
*     * Neither the name of Helix Re Inc. nor the
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

#include <pdal/util/Algorithm.hpp>

#include "E57Writer.hpp"
#include "Utils.hpp"
#include "Uuid.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "writers.e57",
    "E57 format support.",
    "http://libe57.org/"
};

CREATE_SHARED_STAGE(E57Writer, s_info)

E57Writer::ChunkWriter::ChunkWriter
(const std::vector<std::string>& dimensionsToWrite,
 e57::CompressedVectorNode& vectorNode) :
    m_defaultChunkSize(1 << 20), m_currentIndex(0)
{
    // Initialise the write buffers
    for (auto& e57dim: dimensionsToWrite)
        if (!e57dim.empty())
            m_doubleBuffers[e57dim].resize(m_defaultChunkSize);

    for (auto& keyValue : m_doubleBuffers)
        m_e57buffers.emplace_back(vectorNode.destImageFile(), keyValue.first,
             keyValue.second.data(), m_defaultChunkSize, true, true );

    // Setup the writer
    m_dataWriter.reset(
        new e57::CompressedVectorWriter(vectorNode.writer(m_e57buffers)));

}

void E57Writer::ChunkWriter::write(pdal::PointRef& pt)
{
// If buffer full, write to disk and reinitialise buffer

    if (m_currentIndex == m_defaultChunkSize)
    {
        m_dataWriter->write(m_defaultChunkSize);
        m_currentIndex = 0;
    }

    // Add point to buffer and increase index
    for (auto& keyValue: m_doubleBuffers)
    {
        auto pdaldim = pdal::e57plugin::e57ToPdal(keyValue.first);
        if (pdaldim != pdal::Dimension::Id::Unknown)
        {
            auto val = pt.getFieldAs<double>(pdaldim);
            keyValue.second[m_currentIndex] = val;
        }
    }
    m_currentIndex++;
}

void E57Writer::ChunkWriter::finalise()
{
    if (m_dataWriter)
    {
        //  Write whatever remains and closes
        m_dataWriter->write(m_currentIndex);
        m_currentIndex = 0;
        m_dataWriter->close();
    }
}


E57Writer::E57Writer()
{}


E57Writer::~E57Writer()
{
    if (m_imageFile)
        m_imageFile->close();
}

std::string E57Writer::getName() const { return s_info.name; }

void E57Writer::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("doublePrecision",
        "Double precision for storage (false by default)", m_doublePrecision);
}

void E57Writer::initialize()
{
    try 
    {
        m_imageFile.reset(new e57::ImageFile(m_filename, "w"));
        setupFileHeader();
    }
    catch (...) 
    {
        throwError("Could not open file '" + m_filename + "'.");
    }
}

void E57Writer::ready(PointTableRef table)
{
    // Extracts dimensions that can be written
    auto dimensions = table.layout()->dims();
    m_dimensionsToWrite.clear();
    for (auto pdaldim: dimensions)
    {
        std::string e57Dimension(pdal::e57plugin::pdalToE57(pdaldim));
        if (!e57Dimension.empty())
            m_dimensionsToWrite.push_back(e57Dimension);
    }
    setupWriter();
}

void E57Writer::write(const PointViewPtr view)
{
    for (PointId id = 0; id < view->size(); ++id)
    {
        PointRef point = view->point(id);
        processOne(point);
    }
}

bool E57Writer::processOne(PointRef& point)
{
    // Update bounding box
    m_bbox.grow(point.getFieldAs<double>(Dimension::Id::X),
         point.getFieldAs<double>(Dimension::Id::Y),
         point.getFieldAs<double>(Dimension::Id::Z));

    // Write point
    m_chunkWriter->write(point);
    return true;
}

void E57Writer::done(PointTableRef table)
{
    if (m_chunkWriter)
    {
        m_chunkWriter->finalise();
    }
    
    // Set bounding boxes on case by case basis
    if (Utils::contains(m_dimensionsToWrite, "colorRed"))
    {
        // found color info
        using namespace Dimension;

        e57::StructureNode colorbox = e57::StructureNode(*m_imageFile);
        std::vector<Id> colors { Id::Red, Id::Green, Id::Blue };
            
        for (auto id : colors)
        {
            std::string name = Dimension::name(id);
            auto minmax = e57plugin::getPdalBounds(id);
            colorbox.set("color" + name + "Minimum",
                e57::IntegerNode(*m_imageFile,minmax.first));
            colorbox.set("color" + name + "Maximum",
                e57::IntegerNode(*m_imageFile, minmax.second));
        }
        m_scanNode->set("colorLimits", colorbox);
    }
    
    if (Utils::contains(m_dimensionsToWrite, "intensity"))
    {
        // found intensity info
        e57::StructureNode colorbox = e57::StructureNode(*m_imageFile);
        auto minmax = e57plugin::getPdalBounds(Dimension::Id::Intensity);
        colorbox.set("intensityMinimum",
            e57::IntegerNode(*m_imageFile, minmax.first));
        colorbox.set("intensityMaximum",
            e57::IntegerNode(*m_imageFile, minmax.second));
        m_scanNode->set("intensityLimits", colorbox);
    }

    // Cartesian bounds
    e57::StructureNode bboxNode = e57::StructureNode(*m_imageFile);
    bboxNode.set("xMinimum", e57::FloatNode(*m_imageFile, m_bbox.minx));
    bboxNode.set("xMaximum", e57::FloatNode(*m_imageFile, m_bbox.maxx));
    bboxNode.set("yMinimum", e57::FloatNode(*m_imageFile, m_bbox.miny));
    bboxNode.set("yMaximum", e57::FloatNode(*m_imageFile, m_bbox.maxy));
    bboxNode.set("zMinimum", e57::FloatNode(*m_imageFile, m_bbox.minz));
    bboxNode.set("zMaximum", e57::FloatNode(*m_imageFile, m_bbox.maxz));
    m_scanNode->set("cartesianBounds", bboxNode);
    
}

void E57Writer::setupFileHeader()
{
    m_rootNode.reset(new e57::StructureNode(m_imageFile->root()));

    //header info
    // We are using the E57 v1.0 data format standard fieldnames.
    // The standard fieldnames are used without an extension prefix
    // (in the default namespace).
    m_imageFile->extensionsAdd("", E57_V1_0_URI);
    m_imageFile->extensionsAdd("nor",
        "http://www.libe57.org/E57_NOR_surface_normals.txt");

    // Set per-file properties.
    // Path names: "/formatName", "/majorVersion", "/minorVersion",
    // "/coordinateMetadata"
    m_rootNode->set("formatName",
        e57::StringNode(*m_imageFile, "ASTM E57 3D Imaging Data File"));
    m_rootNode->set("guid",
        e57::StringNode(*m_imageFile, uuidGenerator::generate_uuid()));

    // Get ASTM version number supported by library, so can write it into file
    int astmMajor;
    int astmMinor;
    e57::ustring libraryId;
    e57::Utilities::getVersions(astmMajor, astmMinor, libraryId);

    m_rootNode->set("versionMajor", e57::IntegerNode(*m_imageFile, astmMajor));
    m_rootNode->set("versionMinor", e57::IntegerNode(*m_imageFile, astmMinor));
    m_rootNode->set("e57LibraryVersion",
        e57::StringNode(*m_imageFile, libraryId));

    // Save a dummy string for coordinate system.
    m_rootNode->set("coordinateMetadata", e57::StringNode(*m_imageFile, ""));

    // Create creationDateTime structure TODO add actual time
    e57::StructureNode creationDateTime = e57::StructureNode(*m_imageFile);
    creationDateTime.set("dateTimeValue", e57::FloatNode(*m_imageFile, 0.0));
    creationDateTime.set("isAtomicClockReferenced",
        e57::IntegerNode(*m_imageFile, 0));
    m_rootNode->set("creationDateTime", creationDateTime);

//ABELL - This description seems too specific.
    m_rootNode->set("description",
        e57::StringNode(*m_imageFile,"E57 file generated by PDAL"));
}

void E57Writer::setupWriter()
{
    // Create the scan node itself. It will contain a CompressedVectorNode
    // that will store the points
    m_scanNode.reset(new e57::StructureNode(*m_imageFile));
    m_scanNode->set("guid",
        e57::StringNode(*m_imageFile, uuidGenerator::generate_uuid()));

    // Prototype and buffer arrays for the CompressedVectorNodeWriter
    e57::StructureNode proto = e57::StructureNode(*m_imageFile);
    e57::FloatPrecision precision =
        m_doublePrecision ? e57::E57_DOUBLE : e57::E57_SINGLE;
    for (auto& e57Dimension: m_dimensionsToWrite)
    {
        if (e57Dimension.find("color") != std::string::npos)
        {
            auto bounds = e57plugin::getPdalBounds(pdal::Dimension::Id::Red);
            proto.set(e57Dimension,
                e57::IntegerNode(*m_imageFile, 0, bounds.first,bounds.second));
        }
        else if (e57Dimension.find("intensity") != std::string::npos)
        {
            auto bounds = pdal::e57plugin::getPdalBounds(
                pdal::Dimension::Id::Intensity);
            proto.set(e57Dimension,
                e57::IntegerNode(*m_imageFile, 0, bounds.first,bounds.second));
        }
        else
            proto.set(e57Dimension,
                e57::FloatNode(*m_imageFile, 0.0, precision));
    }

	// Create CompressedVector for storing points.
    e57::VectorNode codecs = e57::VectorNode(*m_imageFile, true);
	e57::CompressedVectorNode points =
        e57::CompressedVectorNode(*m_imageFile, proto, codecs);

    // Add the points to the file hierarchy
    m_scanNode->set("points", points);
    e57::VectorNode data3D(*m_imageFile, true);
    m_rootNode->set("data3D", data3D);
    data3D.append(*m_scanNode);

    // Instantiate writer
    try
    {
        m_chunkWriter.reset(new ChunkWriter(m_dimensionsToWrite, points));
    }
    catch (e57::E57Exception &e)
    {
        std::string msg = "E57 error with code " +
            std::to_string(e.errorCode()) + " - " + e.context();
        throwError(msg);
    }
}

} // namespace pdal;
