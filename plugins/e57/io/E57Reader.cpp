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

#include "E57Reader.hpp"
#include "Utils.hpp"
#include "arbiter/arbiter.hpp"
#include <pdal/util/Algorithm.hpp>

namespace pdal
{
using namespace e57;
static PluginInfo const s_info
{
    "readers.e57",
    "Reader for E57 files",
    "http://pdal.io/stages/reader.e57.html"
};

CREATE_SHARED_STAGE(E57Reader, s_info)

std::string E57Reader::getName() const
{
    return s_info.name;
}

E57Reader::E57Reader()
    : Reader(), Streamable()
{
}

void E57Reader::addArgs(ProgramArgs& args)
{
    args.add("extra_dims", "Extra dimensions to read from E57 point cloud.",
             m_extraDimsSpec);
}

void E57Reader::initializeBuffers()
{
    m_doubleBuffers.clear();
    m_destBuffers.clear();
    m_e57PointPrototype.reset(new StructureNode(m_scan->getPointPrototype()));

    // Initialize for supported dimensions.
    auto supportedFields = e57plugin::supportedE57Types();
    for (auto& dimension : supportedFields)
    {
        if (m_e57PointPrototype->isDefined(dimension))
        {
            m_doubleBuffers[dimension] =
                std::vector<double>(m_defaultChunkSize, 0);
        }
    }

    // Initialize for extra dimensions.
    for (auto i = m_extraDims->begin(); i != m_extraDims->end(); ++i)
    {
        if (m_e57PointPrototype->isDefined(i->m_name))
        {
            m_doubleBuffers[i->m_name] =
                std::vector<double>(m_defaultChunkSize, 0);
        }
    }

    // Link to destination buffers.
    for (auto& keyValue : m_doubleBuffers)
    {
        m_destBuffers.emplace_back(
            *m_imf, keyValue.first, keyValue.second.data(), m_defaultChunkSize,
            true,
            (m_e57PointPrototype->get(keyValue.first).type() ==
             e57::E57_SCALED_INTEGER));
    }
}

void E57Reader::addDimensions(PointLayoutPtr layout)
{
    auto supportedFields = e57plugin::supportedE57Types();
    auto numScans = m_data3D->childCount();
    std::unique_ptr<Scan> tempScan;

    for (auto& dimension : supportedFields)
    {
        // Check if any of the scan have this dimension.
        // If any of the scan have this dimension, we cannot ignore it.
        for (auto p = 0; p < numScans; ++p)
        {
            tempScan.reset(new Scan((StructureNode)m_data3D->get(p)));
            auto proto = tempScan->getPointPrototype();
            if (proto.isDefined(dimension))
            {
                layout->registerDim(e57plugin::e57ToPdal(dimension));
                break;
            }
        }
    }

    m_extraDims.reset(new e57plugin::ExtraDims());
    m_extraDims->parse(m_extraDimsSpec);
    auto i = m_extraDims->begin();
    while (i != m_extraDims->end())
    {
        i->m_id = Dimension::Id::Unknown;
        // Remove extra dims which are already in layout.
        if (layout->hasDim(e57plugin::e57ToPdal(i->m_name)))
        {
            i = m_extraDims->deleteDim(i);
            continue;
        }

        // Check if any of the scan have this dimension.
        // If any of the scan have this dimension, we cannot ignore it.
        for (auto p = 0; p < numScans; ++p)
        {
            tempScan.reset(new Scan((StructureNode)m_data3D->get(p)));
            auto proto = tempScan->getPointPrototype();
            if (proto.isDefined(i->m_name))
            {
                i->m_id = layout->registerOrAssignDim(i->m_name, i->m_type);
                break;
            }
        }

        if (i->m_id == Dimension::Id::Unknown)
        {
            // Input E57 point point cloud do not have this dimension. It should be ignored.
            log()->get(LogLevel::Warning) << "Extra dimension specified in pipeline don't match in E57 prototype."
                                          " Ignoring pipeline-specified dimension : " << i->m_name << std::endl;
            i = m_extraDims->deleteDim(i);
            continue;
        }
        ++i;
    }
}

void E57Reader::initialize()
{
    try
    {
        arbiter::Arbiter arb;
        auto fileHandle = arb.getLocalHandle(m_filename);
        m_imf.reset(new ImageFile(fileHandle->localPath(), "r"));
        StructureNode root = m_imf->root();

        if (!root.isDefined("/data3D"))
        {
            throwError("File doesn't contain 3D data");
        }

        const e57::ustring normalsExtension(
            "http://www.libe57.org/E57_NOR_surface_normals.txt");
        e57::ustring _normalsExtension;

        // the extension may already be registered
        if (!m_imf->extensionsLookupPrefix("nor", _normalsExtension))
            m_imf->extensionsAdd("nor", normalsExtension);

        m_data3D.reset(new VectorNode(root.get("/data3D")));

    }
    catch (E57Exception& e)
    {
        throwError(std::to_string(e.errorCode()) + " : " + e.context());
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }
}

void E57Reader::ready(PointTableRef& ref)
{
    log()->get(LogLevel::Info) << "Reading : " << m_filename;
    
    m_currentIndex = 0;
    m_pointsInCurrentBatch = 0;
    m_defaultChunkSize = 10000;
    m_currentScan = -1;
    
    // Initial reader setup.
    setupReader();
}

QuickInfo E57Reader::inspect()
{
    QuickInfo qi;
    std::unique_ptr<PointLayout> layout(new PointLayout());
    initialize();
    addDimensions(layout.get());

    Dimension::IdList dims = layout->dims();
    for (auto di = dims.begin(); di != dims.end(); ++di)
        qi.m_dimNames.push_back(layout->dimName(*di));
    qi.m_pointCount = e57plugin::numPoints(*m_data3D);

    auto numScans = m_data3D->childCount();
    for (int i = 0; i < numScans; ++i)
    {
        Scan scan((StructureNode)m_data3D->get(i));
        qi.m_bounds.grow(scan.getBoundingBox());
    }

    qi.m_valid = true;
    return qi;
}

/// Setup reader to read next scan if available.
void E57Reader::setupReader()
{
    // Are we done with reading all scans?
    if (++m_currentScan >= m_data3D->childCount())
        return;

    try
    {
        m_scan.reset(new Scan((StructureNode)m_data3D->get(m_currentScan)));
        initializeBuffers();
        m_reader.reset(new CompressedVectorReader(
                           m_scan->getPoints().reader(m_destBuffers)));
    }
    catch (E57Exception& e)
    {
        throwError(std::to_string(e.errorCode()) + " : " + e.context());
    }
    catch (...)
    {
        throwError("Got an unknown exception");
    }
}

/// Read the next batch of m_defaultChunkSize.
/// This returns number of points aquired.
/// Returns 0 after finished reading of all scans.
point_count_t E57Reader::readNextBatch()
{
    m_currentIndex = 0;

    // Are we done with reading all scans?
    if (m_currentScan >= m_data3D->childCount())
        return 0;

    point_count_t gotPoints = m_reader->read(m_destBuffers);

    if (!gotPoints)
    {
        // Finished reading all points in current scan.
        // Its time to setup reader at next scan.
        m_reader->close();
        setupReader();
        return readNextBatch();
    }

    return gotPoints;
}

/// Fill the point information.
bool E57Reader::fillPoint(PointRef& point)
{
    if (m_currentIndex >= m_pointsInCurrentBatch)
    {
        // Either we are at very begining or finished processing all points in
        // current batch. Its time to read new points batch.
        m_pointsInCurrentBatch = readNextBatch();
    }

    if (!m_pointsInCurrentBatch)
    {
        // We're done with reading
        return false;
    }

    for (auto& keyValue : m_doubleBuffers)
    {
        auto dim = e57plugin::e57ToPdal(keyValue.first);

        if (dim != Dimension::Id::Unknown)
        {
            point.setField(dim, m_scan->rescale(dim, keyValue.second[m_currentIndex]));
        }
        else
        {
            auto dim = m_extraDims->findDim(keyValue.first);
            if (dim != m_extraDims->end())
            {
                point.setField(dim->m_id, keyValue.second[m_currentIndex]);
            }
        }
    }

    if (m_scan->hasPose())
        m_scan->transformPoint(point);

    ++m_currentIndex;
    return true;
}

point_count_t E57Reader::read(PointViewPtr view, point_count_t count)
{
    point_count_t numPoints = e57plugin::numPoints(*m_data3D);
    for (PointId counter = 0, nextId = view->size(); counter < numPoints;
            ++counter, ++nextId)
    {
        PointRef point(view->point(nextId));
        fillPoint(point);
    }

    return view->size();
}

bool E57Reader::processOne(PointRef& point)
{
    return fillPoint(point);
}

void E57Reader::done(PointTableRef table)
{
    m_imf->close();
}

} // namespace pdal
