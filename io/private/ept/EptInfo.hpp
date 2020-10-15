/******************************************************************************
 * Copyright (c) 2018, Connor Manning
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <pdal/DimType.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/util/Bounds.hpp>

#include "FixedPointLayout.hpp"

namespace pdal
{

class Connector;

class EptInfo
{
public:
    enum class DataType
    {
        Laszip,
        Binary,
        Zstandard
    };

    EptInfo(const std::string& info);
    EptInfo(const std::string& filename, const Connector& connector);

    const BOX3D& bounds() const { return m_bounds; }
    const BOX3D& boundsConforming() const { return m_boundsConforming; }
    uint64_t points() const { return m_points; }
    uint64_t span() const { return m_span; }
    DataType dataType() const { return m_dataType; }
    const SpatialReference& srs() const { return m_srs; }
    const NL::json& json() { return m_info; }
    std::map<std::string, DimType>& dims() { return m_dims; }
    DimType dimType(Dimension::Id id) const;
    PointLayout& remoteLayout() const { return m_remoteLayout; }
    std::string dataDir() const;
    std::string hierarchyDir() const;
    std::string sourcesDir() const;

private:
    // Info comes from the values here:
    // https://entwine.io/entwine-point-tile.html#ept-json
    NL::json m_info;
    BOX3D m_bounds;
    BOX3D m_boundsConforming;
    uint64_t m_points = 0;
    std::map<std::string, DimType> m_dims;

    // The span is the length, width, and depth of the octree grid.  For
    // example, a dataset oriented as a 256*256*256 octree grid would have a
    // span of 256.
    //
    // See: https://entwine.io/entwine-point-tile.html#span
    uint64_t m_span = 0;
    DataType m_dataType;
    SpatialReference m_srs;
    std::string m_filename;
    mutable FixedPointLayout m_remoteLayout;

    void initialize();
};

} // namespace pdal

