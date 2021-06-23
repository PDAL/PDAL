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

#pragma once

#include "VoxelKey.hpp"

#include <unordered_map>

namespace pdal
{
namespace ept
{

class CellManager
{
public:
    using Cells = std::unordered_map<VoxelKey, PointViewPtr>;
    using iterator = Cells::iterator;
    using const_iterator = Cells::const_iterator;

    CellManager(PointViewPtr sourceView) : m_sourceView(sourceView)
    {}

    PointViewPtr& get(VoxelKey key)
    {
        PointViewPtr& v = m_cells[key];
        if (!v)
            v = m_sourceView->makeNew();
        return v;
    }

    iterator erase(iterator it)
    {
        return m_cells.erase(it);
    }

    const_iterator begin() const
    {
        return m_cells.begin();
    }

    const_iterator end() const
    {
        return m_cells.end();
    }

    iterator begin()
    {
        return m_cells.begin();
    }

    iterator end()
    {
        return m_cells.end();
    }

    // Note that C++17 has functions to do this.
    void merge(CellManager&& src)
    {
        for (auto& kv : src)
            m_cells.insert(std::move(kv));
    }

private:
    Cells m_cells;

    PointViewPtr m_sourceView;
};

} // namespace ept
} // namespace pdal
