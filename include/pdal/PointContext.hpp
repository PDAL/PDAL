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

#pragma once

#include <map>
#include <memory>
#include <vector>

#include <boost/property_tree/json_parser.hpp>

#include "pdal/Dimension.hpp"
#include "pdal/Metadata.hpp"
#include "pdal/RawPtBuf.hpp"

namespace pdal
{

class PointBuffer;
class PointContext;
namespace plang
{
    class BufferedInvocation;
}

class DimInfo
{
   friend class PointContext;
public:
    DimInfo() : m_detail(Dimension::COUNT), m_nextFree(Dimension::PROPRIETARY)
        {}

    std::vector<Dimension::Detail> m_detail;
    Dimension::IdList m_used;
    std::map<std::string, Dimension::Id::Enum> m_propIds;
    int m_nextFree;
};
typedef std::shared_ptr<DimInfo> DimInfoPtr;

// This provides a context for processing a set of points and allows the library
// to be used to process multiple point sets simultaneously.
class PointContext
{
    friend class PointBuffer;
    friend class plang::BufferedInvocation;
private:
    DimInfoPtr m_dims;
    // Provides storage for the point data.
    RawPtBufPtr m_ptBuf;
    // Metadata storage;
    MetadataPtr m_metadata;

public:
    PointContext() : m_dims(new DimInfo()), m_ptBuf(new RawPtBuf()),
        m_metadata(new Metadata)
    {}

    RawPtBuf *rawPtBuf() const
        { return m_ptBuf.get(); }
    MetadataNode metadata()
        { return m_metadata->getNode(); }
    SpatialReference spatialRef() const
    {
        MetadataNode m = m_metadata->m_private.findChild("spatialreference");
        SpatialReference sref;
        sref.setWKT(m.value());
        return sref;
    }
    void setSpatialRef(const SpatialReference& sref)
    {
        MetadataNode mp = m_metadata->m_private;
        mp.addOrUpdate("spatialreference", sref.getRawWKT());
    }

    void registerDims(std::vector<Dimension::Id::Enum> ids)
    {
        for (auto ii = ids.begin(); ii != ids.end(); ++ii)
            registerDim(*ii);
    }

    void registerDims(Dimension::Id::Enum *id)
    {
        while (*id != Dimension::Id::Unknown)
            registerDim(*id++);
    }

    void registerDim(Dimension::Id::Enum id)
    {
        registerDim(id, Dimension::defaultType(id));
    }

    void registerDim(Dimension::Id::Enum id, Dimension::Type::Enum type)
    {
        Dimension::Detail& dd = m_dims->m_detail[id];
        if (dd.type() == Dimension::Type::None)
            m_dims->m_used.push_back(id);
        dd.m_type = resolveType(type, dd.m_type);
        update();
    }

    // The type and size are REQUESTS, not absolutes.  If someone else
    // has already registered with the same name, you get the existing
    // dimension size/type.
    Dimension::Id::Enum assignDim(const std::string& name,
        Dimension::Type::Enum type)
    {
        Dimension::Id::Enum id;

        auto di = m_dims->m_propIds.find(name);
        if (di == m_dims->m_propIds.end())
        {
            id = (Dimension::Id::Enum)m_dims->m_nextFree++;
            m_dims->m_propIds[name] = id;
            m_dims->m_used.push_back(id);
        }
        else
            id = di->second;
        m_dims->m_detail[id].m_type =
            resolveType(m_dims->m_detail[id].m_type, type);
        update();
        return id;
    }

    Dimension::Id::Enum registerOrAssignDim(const std::string name,
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

    Dimension::Id::Enum findDim(const std::string& name)
    {
        Dimension::Id::Enum id = Dimension::id(name);
        if (id != Dimension::Id::Unknown)
            return id;
        auto di = m_dims->m_propIds.find(name);
        return (di != m_dims->m_propIds.end() ? di->second :
            Dimension::Id::Unknown);
    }

    std::string dimName(Dimension::Id::Enum id) const
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

    bool hasDim(Dimension::Id::Enum id) const
        { return m_dims->m_detail[id].m_type != Dimension::Type::None; }

    const Dimension::IdList& dims() const
        { return m_dims->m_used; }

    Dimension::Type::Enum dimType(Dimension::Id::Enum id) const
    {
        return hasDim(id) ? dimDetail(id)->type() : Dimension::Type::None;
    }

    std::string dimsJson() const
    {
        boost::property_tree::ptree tree;
        boost::property_tree::ptree dimsTree;

        for (const auto& id : dims())
        {
            boost::property_tree::ptree dim;
            dim.put("name", Dimension::name(id));
            dim.put("type", Dimension::toName(dimDetail(id)->base()));
            dim.put("size", dimDetail(id)->size());
            dimsTree.push_back(std::make_pair("", dim));
        }

        tree.add_child("dimensions", dimsTree);

        std::ostringstream oss;
        boost::property_tree::write_json(oss, tree);
        return oss.str();
    }

private:
    Dimension::Detail *dimDetail(Dimension::Id::Enum id) const
        { return &(m_dims->m_detail[(size_t)id]); }

    void update()
    {
        auto sorter = [this](const Dimension::Id::Enum& d1,
            const Dimension::Id::Enum& d2) -> bool
        {
            size_t s1 = m_dims->m_detail[d1].size();
            size_t s2 = m_dims->m_detail[d2].size();
            if (s1 > s2)
                return true;
            if (s1 < s2)
                return false;
            return d1 < d2;
        };

        Dimension::IdList& used = m_dims->m_used;
        std::sort(used.begin(), used.end(), sorter);
        int offset = 0;
        for (auto ui = used.begin(); ui != used.end(); ++ui)
        {
            m_dims->m_detail[*ui].m_offset = offset;
            offset += m_dims->m_detail[*ui].size();
        }
        m_ptBuf->setPointSize((size_t)offset);
    }

    Dimension::Type::Enum resolveType(Dimension::Type::Enum t1,
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
};

} //namespace

