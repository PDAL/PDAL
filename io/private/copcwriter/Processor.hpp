/*****************************************************************************
 *   Copyright (c) 2020, Hobu, Inc. (info@hobu.co)                           *
 *                                                                           *
 *   All rights reserved.                                                    *
 *                                                                           *
 *   This program is free software; you can redistribute it and/or modify    *
 *   it under the terms of the GNU General Public License as published by    *
 *   the Free Software Foundation; either version 3 of the License, or       *
 *   (at your option) any later version.                                     *
 *                                                                           *
 ****************************************************************************/

#pragma once

#include <pdal/PointView.hpp>

#include "Common.hpp"
#include "VoxelInfo.hpp"

namespace pdal
{
namespace copcwriter
{

class GridKey;
class OctantInfo;
class PyramidManager;

class Processor
{
public:
    Processor(PyramidManager& manager, const VoxelInfo&, const BaseInfo& b);

    void run();

private:
    void sample();
    bool acceptable(GridKey key);
    void writeCompressed(VoxelKey k, PointViewPtr v);

    VoxelInfo m_vi;
    const BaseInfo& b;
    PyramidManager& m_manager;
    las::LoaderDriver m_loader;
};

} // namespace copcwriter
} // namespace pdal
