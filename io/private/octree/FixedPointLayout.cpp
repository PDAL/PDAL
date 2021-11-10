/******************************************************************************
 * Copyright (c) 2020, Hobu Inc.
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

#include <pdal/util/Algorithm.hpp>

#include "FixedPointLayout.hpp"

namespace pdal
{

bool FixedPointLayout::update(Dimension::Detail dimDetail,
    const std::string& name)
{
    if (m_finalized)
        return m_propIds.count(name);

    if (!Utils::contains(m_used, dimDetail.id()))
    {
        dimDetail.setOffset(m_pointSize);

        m_pointSize += dimDetail.size();
        m_used.push_back(dimDetail.id());
        m_detail[Utils::toNative(dimDetail.id())] = dimDetail;

        return true;
    }

    return false;
}

void FixedPointLayout::registerFixedDim(const Dimension::Id id,
    const Dimension::Type type)
{
    Dimension::Detail dd = m_detail[Utils::toNative(id)];
    dd.setType(type);
    update(dd, Dimension::name(id));
}

Dimension::Id FixedPointLayout::registerOrAssignFixedDim(const std::string name,
    const Dimension::Type type)
{
    Dimension::Id id = Dimension::id(name);
    if (id != Dimension::Id::Unknown)
    {
        registerFixedDim(id, type);
        return id;
    }
    return assignDim(name, type);
}

} // namespace pdal

