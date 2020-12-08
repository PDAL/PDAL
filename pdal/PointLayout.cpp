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

#include <pdal/PointLayout.hpp>
#include <pdal/util/Algorithm.hpp>

namespace pdal
{

PointLayout::PointLayout()
    : m_detail(Dimension::COUNT)
    , m_used()
    , m_propIds()
    , m_nextFree(Dimension::PROPRIETARY)
    , m_pointSize(0)
    , m_finalized(false)
{
    int id = 0;
    for (auto& d : m_detail)
    {
        d.setId((Dimension::Id)id);
        id++;
    }
}


void PointLayout::finalize()
{
    m_finalized = true;
}


void PointLayout::registerDims(std::vector<Dimension::Id> ids)
{
    for (auto ii = ids.begin(); ii != ids.end(); ++ii)
        registerDim(*ii);
}


void PointLayout::registerDims(Dimension::Id *id)
{
    while (*id != Dimension::Id::Unknown)
        registerDim(*id++);
}


void PointLayout::registerDim(Dimension::Id id)
{
    registerDim(id, Dimension::defaultType(id));
}


void PointLayout::registerDim(Dimension::Id id, Dimension::Type type)
{
    using namespace Dimension;

    Detail dd = m_detail[Utils::toNative(id)];

    // Force X, Y and Z to always be doubles.
    if (id == Id::X || id == Id::Y || id == Id::Z)
        type = Type::Double;
    else
        type = resolveType(type, dd.type());
    dd.setType(type);
    update(dd, Dimension::name(id));
}


Dimension::Id PointLayout::assignDim(const std::string& name,
    Dimension::Type type)
{
    if (!Dimension::nameValid(name))
        throw pdal_error("Can't create dimension with invalid name '" + name + "'.");
    if (m_nextFree == Dimension::COUNT)
        throw pdal_error("No dimension IDs remaining for assignment.");
    Dimension::Id id = (Dimension::Id)m_nextFree;

    auto di = m_propIds.find(name);
    if (di != m_propIds.end())
        id = di->second;
    Dimension::Detail dd = m_detail[Utils::toNative(id)];
    dd.setType(resolveType(type, dd.type()));
    if (update(dd, name))
    {
        if (di == m_propIds.end())
        {
            m_nextFree++;
            m_propIds[name] = id;
        }
        return id;
    }
    return Dimension::Id::Unknown;
}


Dimension::Id PointLayout::registerOrAssignDim(const std::string name,
   Dimension::Type type)
{
    Dimension::Id id = Dimension::id(name);
    if (id != Dimension::Id::Unknown)
    {
        registerDim(id, type);
        return id;
    }
    return assignDim(name, type);
}


DimTypeList PointLayout::dimTypes() const
{
    DimTypeList dimTypes;

    const Dimension::IdList& ids = dims();
    for (auto ii = ids.begin(); ii != ids.end(); ++ii)
        dimTypes.push_back(DimType(*ii, dimType(*ii)));
    return dimTypes;
}


DimType PointLayout::findDimType(const std::string& name) const
{
    Dimension::Id id = findDim(name);
    return DimType(id, dimType(id));
}


Dimension::Id PointLayout::findDim(const std::string& name) const
{
    Dimension::Id id = Dimension::id(name);
    if (dimType(id) != Dimension::Type::None)
        return id;
    return findProprietaryDim(name);
}


Dimension::Id
PointLayout::findProprietaryDim(const std::string& name) const
{
    auto di = m_propIds.find(name);
    return (di != m_propIds.end() ? di->second :
        Dimension::Id::Unknown);
}


std::string PointLayout::dimName(Dimension::Id id) const
{
    std::string name = Dimension::name(id);
    if (!name.empty())
        return name;
    for (auto pi = m_propIds.begin();
            pi != m_propIds.end(); ++pi)
        if (pi->second == id)
            return pi->first;
    return "";
}


bool PointLayout::hasDim(Dimension::Id id) const
{
    return m_detail[Utils::toNative(id)].type() != Dimension::Type::None;
}


const Dimension::IdList& PointLayout::dims() const
{
    return m_used;
}


Dimension::Type PointLayout::dimType(Dimension::Id id) const
{
    return dimDetail(id)->type();
}


size_t PointLayout::dimSize(Dimension::Id id) const
{
    return dimDetail(id)->size();
}


size_t PointLayout::dimOffset(Dimension::Id id) const
{
    return dimDetail(id)->offset();
}


size_t PointLayout::pointSize() const
{
    return m_pointSize;
}


// Update the point layout given dimension detail and the dimension's name.
bool PointLayout::update(Dimension::Detail dd, const std::string& name)
{
    if (m_finalized)
        throw pdal_error("Can't update layout after points have been added.");

    // If we have a list of allowed dimensions and this dimension isn't listed, return false.
    if (m_allowedDimNames.size() && !Utils::contains(m_allowedDimNames, Utils::toupper(name)))
        return false;

    Dimension::DetailList detail;

    // Update the list of used IDs.
    bool used = Utils::contains(m_used, dd.id());
    for (auto id : m_used)
    {
        if (id == dd.id())
            detail.push_back(dd);
        else
            detail.push_back(m_detail[Utils::toNative(id)]);
    }
    if (!used)
        detail.push_back(dd);

    // Find the dimension in the list that we're referring to with
    // this update.
    auto di = std::find_if(detail.begin(), detail.end(),
        [dd](const Dimension::Detail& td){ return td.id() == dd.id(); });
    Dimension::Detail *cur = &(*di);

    {
        auto sorter = [](const Dimension::Detail& d1,
                const Dimension::Detail& d2) -> bool
        {
            if (d1.size() > d2.size())
                return true;
            if (d1.size() < d2.size())
                return false;
            return d1.id() < d2.id();
        };

        // Sort dimensions based on size and then on ID.
        int offset = 0;
        std::sort(detail.begin(), detail.end(), sorter);
        for (auto& d : detail)
        {
            d.setOffset(offset);
            offset += (int)d.size();
        }
        //NOTE - I tried forcing all points to be aligned on 8-byte boundaries
        // in case this would matter to the optimized memcpy, but it made
        // no difference.  No sense wasting space for no difference.
        m_pointSize = (size_t)offset;
    }

    if (!used)
        m_used.push_back(dd.id());

    // Update the detail for the ID.
    for (auto& dtemp : detail)
        m_detail[Utils::toNative(dtemp.id())] = dtemp;

    return true;
}


// Given two types, find a type that can represent the values of both.
Dimension::Type PointLayout::resolveType(Dimension::Type t1,
    Dimension::Type t2)
{
    using namespace Dimension;
    if (t1 == Type::None && t2 != Type::None)
       return t2;
    if (t2 == Type::None && t1 != Type::None)
       return t1;
    if (t1 == t2)
        return t1;
    if (base(t1) == base(t2))
        return (std::max)(t1, t2);
    //Prefer floating to non-floating.
    if (base(t1) == BaseType::Floating && base(t2) != BaseType::Floating)
        return t1;
    if (base(t2) == BaseType::Floating && base(t1) != BaseType::Floating)
        return t2;
    // Now we're left with cases of a signed/unsigned mix.
    // If the unsigned type is smaller, take the signed type.
    if (base(t1) == BaseType::Unsigned && size(t1) < size(t2))
        return t2;
    if (base(t2) == BaseType::Unsigned && size(t2) < size(t1))
        return t1;
    // Signed type is smaller or the the sizes are equal.
    switch ((std::max)(size(t1), size(t2)))
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

MetadataNode PointLayout::toMetadata() const
{

    MetadataNode root("schema");

    for (const auto& id : dims())
    {
        MetadataNode dim("dimensions");
        dim.add("name", dimName(id));
        Dimension::Type t = dimType(id);
        dim.add("type", Dimension::toName(Dimension::base(t)));
        dim.add("size", dimSize(id));
        root.addList(dim);
    }

    return root;
}

void PointLayout::setAllowedDims(StringList dimNames)
{
    if (dimNames.empty())
        return;

    for (std::string& s : dimNames)
        s = Utils::toupper(s);
    for (const std::string& xyz : { "X", "Y", "Z" })
        if (!Utils::contains(dimNames, xyz))
            dimNames.push_back(xyz);
    m_allowedDimNames = dimNames;
}

} // namespace pdal


