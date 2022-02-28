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

#pragma once

#include <pdal/PointView.hpp>

#include "VoxelKey.hpp"

#include <unordered_map>

namespace pdal
{
namespace copcwriter
{

// This simply holds the point view for each hierarchy entry.
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

    void merge(CellManager& src)
    {
        m_cells.merge(std::move(src.m_cells));
    }

private:
    Cells m_cells;
    PointViewPtr m_sourceView;
};

} // namespace copcwriter
} // namespace pdal
