/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include "DeltaKernel.hpp"

#include <pdal/PDALUtils.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.delta",
    "Delta Kernel",
    "http://pdal.io/kernels/kernels.delta.html" );

CREATE_STATIC_PLUGIN(1, 0, DeltaKernel, Kernel, s_info)

std::string DeltaKernel::getName() const { return s_info.name; }

DeltaKernel::DeltaKernel() : m_3d(true), m_detail(false), m_allDims(false)
{}


void DeltaKernel::addSwitches()
{
    namespace po = boost::program_options;

    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
        ("source", po::value<std::string>(&m_sourceFile),
         "source file name")
        ("candidate", po::value<std::string>(&m_candidateFile),
         "candidate file name")
        ("output", po::value<std::string>(&m_outputFile),
         "output file name")
        ("2d", po::value<bool>(&m_3d)->zero_tokens()->implicit_value(false),
         "only 2D comparisons/indexing")
        ("detail",
         po::value<bool>(&m_detail)->zero_tokens()->implicit_value(true),
         "Output deltas per-point")
        ("alldims",
         po::value<bool>(&m_allDims)->zero_tokens()->implicit_value(true),
         "Compute diffs for all dimensions (not just X,Y,Z)");
    addSwitchSet(file_options);

    po::options_description* processing_options =
        new po::options_description("processing options");

    processing_options->add_options();
    addSwitchSet(processing_options);

    addPositionalSwitch("source", 1);
    addPositionalSwitch("candidate", 2);
    addPositionalSwitch("output", 3);
}


PointViewPtr DeltaKernel::loadSet(const std::string& filename,
    PointTable& table)
{
    Options ops;

    ops.add<std::string>("filename", filename);
    ops.add<bool>("debug", isDebug());
    ops.add<uint32_t>("verbose", getVerboseLevel());

    Stage& reader = makeReader(filename);
    reader.setOptions(ops);
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    assert(viewSet.size() == 1);
    return *viewSet.begin();
}


int DeltaKernel::execute()
{
    PointTable srcTable;
    PointTable candTable;
    DimIndexMap dims;

    PointViewPtr srcView = loadSet(m_sourceFile, srcTable);
    PointViewPtr candView = loadSet(m_candidateFile, candTable);

    PointLayoutPtr srcLayout = srcTable.layout();
    PointLayoutPtr candLayout = candTable.layout();

    Dimension::IdList ids = srcLayout->dims();
    for (Dimension::Id::Enum dim : ids)
    {
        std::string name = srcLayout->dimName(dim);
        if (!m_allDims)
            if (name != "X" && name != "Y" && name != "Z")
                continue;
        DimIndex d;
        d.m_name = name;
        d.m_srcId = dim;
        dims[name] = d;
    }
    ids = candLayout->dims();
    for (Dimension::Id::Enum dim : ids)
    {
        std::string name = candLayout->dimName(dim);
        auto di = dims.find(name);
        if (di == dims.end())
            continue;
        DimIndex& d = di->second;
        d.m_candId = dim;
    }

    // Remove dimensions that aren't in both the source and candidate lists.
    for (auto di = dims.begin(); di != dims.end();)
    {
        DimIndex& d = di->second;
        if (d.m_candId == Dimension::Id::Unknown)
            dims.erase(di++);
        else
            ++di;
    }

    // Index the candidate data.
    KD3Index index(*candView);
    index.build();

    MetadataNode root;

    if (m_detail)
        root = dumpDetail(srcView, candView, index, dims);
    else
        root = dump(srcView, candView, index, dims);
    Utils::toJSON(root, std::cout);

    return 0;
}


MetadataNode DeltaKernel::dump(PointViewPtr& srcView, PointViewPtr& candView,
    KD3Index& index, DimIndexMap& dims)
{
    MetadataNode root;

    for (PointId id = 0; id < srcView->size(); ++id)
    {
        double x = srcView->getFieldAs<double>(Dimension::Id::X, id);
        double y = srcView->getFieldAs<double>(Dimension::Id::Y, id);
        double z = srcView->getFieldAs<double>(Dimension::Id::Z, id);

        PointId candId = index.neighbor(x, y, z);

        // It may be faster to put in a special case to avoid having to
        // fetch X, Y and Z, more than once but this is simpler and
        // I'm thinking in most cases it will make no practical difference.
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            DimIndex& d = di->second;
            double sv = srcView->getFieldAs<double>(d.m_srcId, id);
            double cv = candView->getFieldAs<double>(d.m_candId, candId);
            accumulate(d, sv - cv);
        }
    }

    root.add("source", m_sourceFile);
    root.add("candidate", m_candidateFile);
    for (auto dpair : dims)
    {
        DimIndex& d = dpair.second;

        MetadataNode dimNode = root.add(d.m_name);
        dimNode.add("min", d.m_min);
        dimNode.add("max", d.m_max);
        dimNode.add("mean", d.m_avg);
    }
    return root;
}


void DeltaKernel::accumulate(DimIndex& d, double v)
{
    d.m_cnt++;
    d.m_min = std::min(v, d.m_min);
    d.m_max = std::max(v, d.m_max);
    d.m_avg += (v - d.m_avg) / d.m_cnt;
}


MetadataNode DeltaKernel::dumpDetail(PointViewPtr& srcView,
    PointViewPtr& candView, KD3Index& index, DimIndexMap& dims)
{
    MetadataNode root;

    for (PointId id = 0; id < srcView->size(); ++id)
    {
        double x = srcView->getFieldAs<double>(Dimension::Id::X, id);
        double y = srcView->getFieldAs<double>(Dimension::Id::Y, id);
        double z = srcView->getFieldAs<double>(Dimension::Id::Z, id);
        PointId candId = index.neighbor(x, y, z);

        MetadataNode delta = root.add("delta");
        delta.add("i", id);
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            DimIndex& d = di->second;
            double sv = srcView->getFieldAs<double>(d.m_srcId, id);
            double cv = candView->getFieldAs<double>(d.m_candId, candId);

            delta.add(d.m_name, sv - cv);
        }
    }
    return root;
}

} // pdal
