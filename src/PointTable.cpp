/******************************************************************************
* Copyright (c) 2014, Hobu Inc.
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

#include <pdal/PointTable.hpp>

namespace pdal
{

PointTable::PointTable()
    : m_dims(new DimInfo())
    , m_ptBuf(new DefaultRawPtBuf())
    , m_metadata(new Metadata)
{}

PointTable::PointTable(RawPtBufPtr ptBuf)
    : m_dims(new DimInfo())
    , m_ptBuf(ptBuf)
    , m_metadata(new Metadata)
{}

RawPtBuf* PointTable::rawPtBuf() const
{
    return m_ptBuf.get();
}

MetadataNode PointTable::metadata()
{
    return m_metadata->getNode();
}

SpatialReference PointTable::spatialRef() const
{
    MetadataNode m = m_metadata->m_private.findChild("spatialreference");
    SpatialReference sref;
    sref.setWKT(m.value());
    return sref;
}

void PointTable::setSpatialRef(const SpatialReference& sref)
{
    MetadataNode mp = m_metadata->m_private;
    mp.addOrUpdate("spatialreference", sref.getRawWKT());
}

void PointTable::registerDims(std::vector<Dimension::Id::Enum> ids)
{
    for (auto ii = ids.begin(); ii != ids.end(); ++ii)
        registerDim(*ii);
}

void PointTable::registerDims(Dimension::Id::Enum *id)
{
    while (*id != Dimension::Id::Unknown)
        registerDim(*id++);
}

void PointTable::registerDim(Dimension::Id::Enum id)
{
    registerDim(id, Dimension::defaultType(id));
}

void PointTable::registerDim(Dimension::Id::Enum id, Dimension::Type::Enum type)
{
    Dimension::Detail dd = m_dims->m_detail[id];
    dd.setType(resolveType(type, dd.type()));
    update(dd, Dimension::name(id));
}

Dimension::Id::Enum PointTable::assignDim(const std::string& name,
    Dimension::Type::Enum type)
{
    Dimension::Id::Enum id = (Dimension::Id::Enum)m_dims->m_nextFree;

    auto di = m_dims->m_propIds.find(name);
    if (di != m_dims->m_propIds.end())
        id = di->second;
    Dimension::Detail dd = m_dims->m_detail[id];
    dd.setType(resolveType(type, dd.type()));
    if (update(dd, name))
    {
        if (di == m_dims->m_propIds.end())
        {
            m_dims->m_nextFree++;
            m_dims->m_propIds[name] = id;
        }
        return id;
    }
    return Dimension::Id::Unknown;
}

Dimension::Id::Enum PointTable::registerOrAssignDim(const std::string name,
   Dimension::Type::Enum type)
{
    Dimension::Id::Enum id = Dimension::id(name);
    if (id != Dimension::Id::Unknown)
    {
        registerDim(id, type);
        return id;
    }
    return assignDim(name, type);
}

DimTypeList PointTable::dimTypes() const
{
    DimTypeList dimTypes;

    const Dimension::IdList& ids = dims();
    for (auto ii = ids.begin(); ii != ids.end(); ++ii)
        dimTypes.push_back(DimType(*ii, dimType(*ii)));
    return dimTypes;
}

DimType PointTable::findDimType(const std::string& name) const
{
    Dimension::Id::Enum id = findDim(name);
    return DimType(id, dimType(id));
}

Dimension::Id::Enum PointTable::findDim(const std::string& name) const
{
    Dimension::Id::Enum id = Dimension::id(name);
    if (dimType(id) != Dimension::Type::None)
        return id;
    return findProprietaryDim(name);
}

Dimension::Id::Enum PointTable::findProprietaryDim(const std::string& name) const
{
    auto di = m_dims->m_propIds.find(name);
    return (di != m_dims->m_propIds.end() ? di->second :
        Dimension::Id::Unknown);
}

std::string PointTable::dimName(Dimension::Id::Enum id) const
{
    std::string name = Dimension::name(id);
    if (!name.empty())
        return name;
    for (auto pi = m_dims->m_propIds.begin();
            pi != m_dims->m_propIds.end(); ++pi)
        if (pi->second == id)
            return pi->first;
    return "";
}

bool PointTable::hasDim(Dimension::Id::Enum id) const
{
    return m_dims->m_detail[id].type() != Dimension::Type::None;
}

const Dimension::IdList& PointTable::dims() const
{
    return m_dims->m_used;
}

Dimension::Type::Enum PointTable::dimType(Dimension::Id::Enum id) const
{
    return dimDetail(id)->type();
}

size_t PointTable::dimSize(Dimension::Id::Enum id) const
{
    return dimDetail(id)->size();
}

size_t PointTable::dimOffset(Dimension::Id::Enum id) const
{
    return dimDetail(id)->offset();
}

size_t PointTable::pointSize() const
{
    size_t size(0);
    for (const auto& d : m_dims->m_detail)
        size += d.size();
    return size;
}

Dimension::Detail* PointTable::dimDetail(Dimension::Id::Enum id) const
{
    return &(m_dims->m_detail[(size_t)id]);
}

bool PointTable::update(Dimension::Detail dd, const std::string& name)
{
    Dimension::DetailList detail;

    bool used = Utils::contains(m_dims->m_used, dd.id());
    for (auto id : m_dims->m_used)
    {
        if (id == dd.id())
            detail.push_back(dd);
        else
            detail.push_back(m_dims->m_detail[id]);
    }
    if (!used)
        detail.push_back(dd);

    // Find the dimension in the list that we're referring to with
    // this update.
    auto di = std::find_if(detail.begin(), detail.end(),
        [dd](const Dimension::Detail& td){ return td.id() == dd.id(); });
    Dimension::Detail *cur = &(*di);

    bool addDim = rawPtBuf()->update(detail, cur, name);
    if (addDim)
    {
        if (!used)
            m_dims->m_used.push_back(dd.id());
        for (auto& dtemp : detail)
            m_dims->m_detail[dtemp.id()] = dtemp;
    }
    return addDim;
}

Dimension::Type::Enum PointTable::resolveType(
        Dimension::Type::Enum t1,
        Dimension::Type::Enum t2)
{
    using namespace Dimension;
    using namespace Dimension::BaseType;
    if (t1 == Type::None && t2 != Type::None)
       return t2;
    if (t2 == Type::None && t1 != Type::None)
       return t1;
    if (t1 == t2)
        return t1;
    if (base(t1) == base(t2))
        return std::max(t1, t2);
    //Prefer floating to non-floating.
    if (base(t1) == Floating && base(t2) != Floating)
        return t1;
    if (base(t2) == Floating && base(t1) != Floating)
        return t2;
    // Now we're left with cases of a signed/unsigned mix.
    // If the unsigned type is smaller, take the signed type.
    if (base(t1) == Unsigned && size(t1) < size(t2))
        return t2;
    if (base(t2) == Unsigned && size(t2) < size(t1))
        return t1;
    // Signed type is smaller or the the sizes are equal.
    switch (std::max(size(t1), size(t2)))
    {
    case 1:
        return Type::Signed16;
    case 2:
        return Type::Signed32;
    case 4:
        return Type::Signed64;
    default:
        return Type::Double;
    }
}

} // namespace pdal

