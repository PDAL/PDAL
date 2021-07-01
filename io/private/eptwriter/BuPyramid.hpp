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

#include <string>
#include <unordered_map>
#include <vector>

#include "CellManager.hpp"
#include "Common.hpp"
#include "PyramidManager.hpp"

namespace pdal
{
class ProgramArgs;
class Options;

namespace ept
{

class BuPyramid
{
public:
    BuPyramid(BaseInfo& common);
    void run(CellManager& cells);

private:
    size_t queueWork(CellManager& cells);
    void writeInfo();

    BaseInfo m_b;
    PyramidManager m_manager;
};

} // namespace ept
} // namespace pdal
