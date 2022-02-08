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

#include "CellManager.hpp"
#include "Grid.hpp"

namespace pdal
{
namespace copcwriter
{

class Reprocessor
{
public:
    Reprocessor(CellManager& mgr, PointViewPtr srcView, Grid grid);

    void run();

private:
    int m_levels;
    CellManager& m_mgr;
    PointViewPtr m_srcView;
    Grid m_grid;
};

} // namespace copcwriter
} // namespace pdal
