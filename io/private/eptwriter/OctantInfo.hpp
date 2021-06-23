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

#include "VoxelKey.hpp"

namespace untwine
{
namespace bu
{

class OctantInfo
{
public:
    OctantInfo() : m_mustWrite(false)
    {}

    OctantInfo(const VoxelKey& key) : m_mustWrite(false)
        { m_key = key; }

    void appendSource(PointViewPtr source)
    {
        if (!m_source)
            m_source = source;
        else
            m_source.append(source);
    }

/**
    PointViewPtr newSource() const
    {
        return m_source ? m_source->nameNew() : PointViewPtr();
    }
**/

    size_t numPoints() const
    {
        return m_source ? m_source->size() : 0;
    }

    VoxelKey key() const
        { return m_key; }
    void setKey(VoxelKey k)
        { m_key = k; }
    bool mustWrite() const
        { return m_mustWrite; }
    void setMustWrite(bool mustWrite)
        { m_mustWrite = mustWrite; }

private:
    PointViewPtr m_source;
    VoxelKey m_key;
    bool m_mustWrite;
};

} // namespace bu
} // namespace untwine
