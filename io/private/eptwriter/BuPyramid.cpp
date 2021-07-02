/*****************************************************************************
 *   Copyright (c) 2021, Hobu, Inc. (info@hobu.co)                           *
 *                                                                           *
 *   All rights reserved.                                                    *
 *                                                                           *
 *   This program is free software; you can redistribute it and/or modify    *
 *   it under the terms of the GNU General Public License as published by    *
 *   the Free Software Foundation; either version 3 of the License, or       *
 *   (at your option) any later version.                                     *
 *                                                                           *
 ****************************************************************************/

#include <iomanip>
#include <set>
#include <string>
#include <vector>

#include <pdal/util/FileUtils.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "BuPyramid.hpp"
#include "OctantInfo.hpp"

namespace pdal
{
namespace ept
{

namespace
{

void createDirs(const std::string& outputDir)
{
    //ABELL - Check for errors. createDirectory() returns bool.
    FileUtils::createDirectory(outputDir);
    FileUtils::deleteFile(outputDir + "/ept.json");
    FileUtils::deleteDirectory(outputDir + "/ept-data");
    FileUtils::deleteDirectory(outputDir + "/ept-hierarchy");
    FileUtils::createDirectory(outputDir + "/ept-data");
    FileUtils::createDirectory(outputDir + "/ept-hierarchy");
}

}

/// BuPyramid

BuPyramid::BuPyramid(BaseInfo& common) : m_b(common), m_manager(m_b)
{}


void BuPyramid::run(CellManager& cells)
{
    createDirs(m_b.outputDir);
    queueWork(cells);
    std::thread runner(&PyramidManager::run, &m_manager);
    runner.join();
    writeInfo();
}


void BuPyramid::writeInfo()
{
    auto typeString = [](pdal::Dimension::BaseType b)
    {
        using namespace pdal::Dimension;

        switch (b)
        {
        case BaseType::Signed:
            return "signed";
        case BaseType::Unsigned:
            return "unsigned";
        case BaseType::Floating:
            return "float";
        default:
            return "";
        }
    };

    std::ofstream out(m_b.outputDir + "/ept.json");

    out << "{\n";

    pdal::BOX3D& b = m_b.bounds;

    // Set fixed output for bounds output to get sufficient precision.
    out << std::fixed;
    out << "\"bounds\": [" <<
        b.minx << ", " << b.miny << ", " << b.minz << ", " <<
        b.maxx << ", " << b.maxy << ", " << b.maxz << "],\n";

    pdal::BOX3D& tb = m_b.trueBounds;
    out << "\"boundsConforming\": [" <<
        tb.minx << ", " << tb.miny << ", " << tb.minz << ", " <<
        tb.maxx << ", " << tb.maxy << ", " << tb.maxz << "],\n";
    // Reset to default float output to match PDAL option handling for now.
    out << std::defaultfloat;

    out << "\"dataType\": \"laszip\",\n";
    out << "\"hierarchyType\": \"json\",\n";
    out << "\"points\": " << m_manager.totalPoints() << ",\n";
    out << "\"span\": 128,\n";
    out << "\"version\": \"1.0.0\",\n";
    out << "\"schema\": [\n";

    PointLayoutPtr l = m_b.table.layout();
    Dimension::IdList ids = l->dims();
    for (auto it = ids.begin(); it != ids.end(); ++it)
    {
        Dimension::Id id = *it;
        out << "\t{";
            out << "\"name\": \"" << l->dimName(id) << "\", ";
            int o = -1;
            if (id == Dimension::Id::X)
                o = 0;
            else if (id == Dimension::Id::Y)
                o = 1;
            else if (id == Dimension::Id::Z)
                o = 2;
            // X, Y or Z
            if (o >= 0)
            {
                out << "\"type\": \"signed\", ";
                out << "\"scale\": " << m_b.scale[o] << ", \"offset\": " << m_b.offset[o] << ", ";
                out << "\"size\": 4";
            }
            else
            {
                out << "\"type\": \"" << typeString(Dimension::base(l->dimType(id))) << "\", ";
                out << "\"size\": " << l->dimSize(id);
            }
        out << "}";
        if (it + 1 != ids.end())
            out << ",";
        out << "\n";
    }
    out << "],\n";

    out << "\"srs\": {\n";
    if (m_b.srs.valid())
    {
        out << "\"wkt\": " <<  "\"" << pdal::Utils::escapeJSON(m_b.srs.getWKT()) << "\"\n";
    }
    out << "}\n";

    out << "}\n";
}


size_t BuPyramid::queueWork(CellManager& cells)
{
    std::set<VoxelKey> needed;
    std::set<VoxelKey> parentsToProcess;
    std::vector<OctantInfo> have;
    const VoxelKey root;

    for (auto& kv : cells)
    {
        VoxelKey k = kv.first;
        PointViewPtr v = kv.second;

        // Stick an OctantInfo for this cell in the 'have' list.
        OctantInfo o(k);
        o.appendSource(v);
        have.push_back(o);

        // Walk up the tree and make sure that we're populated for all children necessary
        // to process to the top level.  We do this in order to facilitate processing --
        // a parent node is ready to be processed when all its children have been processed,
        // so making sure a parent has all 8 children makes this easy.
        while (k != root)
        {
            k = k.parent();
            parentsToProcess.insert(k);
            for (int i = 0; i < 8; ++i)
                needed.insert(k.child(i));
        }
    }

    // Now remove entries for the cells we have and their parents.
    for (const OctantInfo& o : have)
    {
        VoxelKey k = o.key();
        while (k != root)
        {
            needed.erase(k);
            k = k.parent();
        }
    }

    // Queue what we have.
    for (const OctantInfo& o : have)
        m_manager.queue(o);

    // Queue what we need but have no data for.
    for (const VoxelKey& k : needed)
        m_manager.queue(OctantInfo(k));
    return parentsToProcess.size();
}

} // namespace ept
} // namespace pdal
