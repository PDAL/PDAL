/******************************************************************************
* Copyright (c) 2021, Hobu Inc. (info@hobu.co)
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

#include "EptWriter.hpp"

#include "private/eptwriter/BuPyramid.hpp"
#include "private/eptwriter/Ept.hpp"
#include "private/eptwriter/CellManager.hpp"
#include "private/eptwriter/Grid.hpp"
#include "private/eptwriter/Reprocessor.hpp"

namespace pdal
{

namespace
{

const StaticPluginInfo s_info
{
    "writers.ept",
    "EPT Writer",
    "http://pdal.io/stages/writer.ept.html",
    { "ept" }
};

}

CREATE_STATIC_STAGE(EptWriter, s_info);

struct EptWriter::Args
{
public:
    std::string outputDir;
};

struct EptWriter::Private
{
public:
};

EptWriter::EptWriter() : m_args(new EptWriter::Args), d(new EptWriter::Private)
{}

EptWriter::~EptWriter()
{}

std::string EptWriter::getName() const { return s_info.name; }

void EptWriter::addArgs(ProgramArgs& args)
{
    args.add("directory", "Directory in which to write the EPT output.", m_args->outputDir);
}


void EptWriter::write(const PointViewPtr v)
{
    using namespace ept;

    BOX3D box;
    size_t count = v->size();
    v->calculateBounds(box);

    Grid grid;
    grid.expand(box, count);

    CellManager mgr(v);

    for (PointRef p : *v)
    {
        double x = p.getFieldAs<double>(Dimension::Id::X);
        double y = p.getFieldAs<double>(Dimension::Id::Y);
        double z = p.getFieldAs<double>(Dimension::Id::Z);

        VoxelKey key = grid.key(x, y, z);
        PointViewPtr& cell = mgr.get(key);
        cell->appendPoint(*v, p.pointId());
    }

    // New cells from reprocessing go on the reprocessing manager. They get merged
    // at the end.
    //ABELL - This should be threaded. These reprocessors should be able to run independently
    // since their data doesn't overlap spatially. Probably need a separate CellManager
    // for each that is merged under lock at completion.
    CellManager reprocessMgr(v);
    auto it = mgr.begin();
    while (it != mgr.end())
    {
        PointViewPtr& v = it->second;
        if (v->size() >= ept::MaxPointsPerNode)
        {
            Reprocessor r(reprocessMgr, v, grid);
            r.run();
            // Remove the reprocessed cell from the manager.
            it = mgr.erase(it);
        }
        else
            it++;
    }
    mgr.merge(std::move(reprocessMgr));

    BaseInfo common(v->table());
    common.bounds = grid.processingBounds();
    common.trueBounds = grid.conformingBounds();
    common.outputDir = m_args->outputDir;
    common.srs = v->spatialReference();
    grid.offset(common.offset);
    grid.scale(common.scale);

    BuPyramid bu(common);
    bu.run(mgr);
}


/**
bool EptWriter::processOne(PointRef& point)
{
    return true;
}
**/

} // namespace pdal
