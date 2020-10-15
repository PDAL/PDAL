/******************************************************************************
* Copyright (c) 2015, Hobu Inc.
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

#include <pdal/PDALUtils.hpp>
#include <pdal/PointContainer.hpp>
#include <pdal/PointLayout.hpp>
#include <pdal/util/Utils.hpp>

namespace pdal
{

class PDAL_DLL PointRef
{
public:
    PointRef() : m_container(nullptr), m_layout(nullptr), m_idx(0), m_tmp(false)
    {}

    PointRef(PointContainer& container, PointId idx = 0) :
        m_container(&container), m_layout(container.layout()), m_idx(idx),
        m_tmp(false)
    {}

    PointRef(const PointRef& r) :
        m_container(r.m_container), m_layout(r.m_layout), m_tmp(true)
    {
        m_idx = m_container->getTemp(r.m_idx);
    }

    ~PointRef()
    {
        if (m_tmp)
            m_container->freeTemp(m_idx);
    }

    PointRef& operator=(const PointRef& r)
    {
        if (!m_container)
        {
            m_container = r.m_container;
            m_layout = r.m_layout;
            m_idx = m_container->getTemp(r.m_idx);
            m_tmp = true;
        }
        else
            m_container->setItem(m_idx, r.m_idx);
        return *this;
    }

    /**
      Determine if the point contains the specified dimension (defers to
      the layout).

      \param dim  Dimension to check for existence.
      \return  Whether the point has a dimension.
    */
    bool hasDim(Dimension::Id dim) const
    { return m_layout->hasDim(dim); }

    /**
      Get the value of a field/dimension, converting it to the type as
      requested.  NOTE: Throws an exception if the value of the dimension
      can't be converted to the requested type.

      \param dim  Dimension to get.
      \return  Value of the dimension.
    */
    template<class T>
    T getFieldAs(Dimension::Id dim) const
    {
        T val(0);
        bool success = true;
        Everything e;
        Dimension::Type type = m_layout->dimDetail(dim)->type();

        // We don't hoist getFieldInternal() out of the switch stmt.
        // because we don't want to call if the type is bad.  We also
        // don't want an extra check on type.
        switch (type)
        {
        case Dimension::Type::Unsigned8:
            m_container->getFieldInternal(dim, m_idx, &e);
            success = Utils::numericCast(e.u8, val);
            break;
        case Dimension::Type::Unsigned16:
            m_container->getFieldInternal(dim, m_idx, &e);
            success = Utils::numericCast(e.u16, val);
            break;
        case Dimension::Type::Unsigned32:
            m_container->getFieldInternal(dim, m_idx, &e);
            success = Utils::numericCast(e.u32, val);
            break;
        case Dimension::Type::Unsigned64:
            m_container->getFieldInternal(dim, m_idx, &e);
            success = Utils::numericCast(e.u64, val);
            break;
        case Dimension::Type::Signed8:
            m_container->getFieldInternal(dim, m_idx, &e);
            success = Utils::numericCast(e.s8, val);
            break;
        case Dimension::Type::Signed16:
            m_container->getFieldInternal(dim, m_idx, &e);
            success = Utils::numericCast(e.s16, val);
            break;
        case Dimension::Type::Signed32:
            m_container->getFieldInternal(dim, m_idx, &e);
            success = Utils::numericCast(e.s32, val);
            break;
        case Dimension::Type::Signed64:
            m_container->getFieldInternal(dim, m_idx, &e);
            success = Utils::numericCast(e.s64, val);
            break;
        case Dimension::Type::Float:
            m_container->getFieldInternal(dim, m_idx, &e);
            success = Utils::numericCast(e.f, val);
            break;
        case Dimension::Type::Double:
            m_container->getFieldInternal(dim, m_idx, &e);
            success = Utils::numericCast(e.d, val);
            break;
        case Dimension::Type::None:
            val = 0;
            break;
        }
        if (!success)
        {
            std::ostringstream oss;
            oss << "Unable to fetch data and convert as requested: ";
            oss << Dimension::name(dim) << ":" <<
                Dimension::interpretationName(type) <<
                "(" << Utils::toDouble(e, type) << ") -> " <<
                Utils::typeidName<T>();
            throw pdal_error(oss.str());
        }
        return val;
    }

    /**
      Set the value of a field/dimension for a point.

      \param dim  Dimension to set.
      \param val  Value to set.
    */
    template<typename T>
    void setField(Dimension::Id dim, T val)
    {
        Dimension::Type type = m_layout->dimDetail(dim)->type();
        Everything e;
        bool success = false;

        switch (type)
        {
        case Dimension::Type::Unsigned8:
            success = Utils::numericCast(val, e.u8);
            break;
        case Dimension::Type::Unsigned16:
            success = Utils::numericCast(val, e.u16);
            break;
        case Dimension::Type::Unsigned32:
            success = Utils::numericCast(val, e.u32);
            break;
        case Dimension::Type::Unsigned64:
            success = Utils::numericCast(val, e.u64);
            break;
        case Dimension::Type::Signed8:
            success = Utils::numericCast(val, e.s8);
            break;
        case Dimension::Type::Signed16:
            success = Utils::numericCast(val, e.s16);
            break;
        case Dimension::Type::Signed32:
            success = Utils::numericCast(val, e.s32);
            break;
        case Dimension::Type::Signed64:
            success = Utils::numericCast(val, e.s64);
            break;
        case Dimension::Type::Float:
            success = Utils::numericCast(val, e.f);
            break;
        case Dimension::Type::Double:
            success = Utils::numericCast(val, e.d);
            break;
        case Dimension::Type::None:
            break;
        }

        if (success)
            m_container->setFieldInternal(dim, m_idx, &e);
    }

    /**
      Set the ID of a PointRef.

      \param idx  point ID
    */
    void setPointId(PointId idx)
        { m_idx = idx; }

    /**
      Fetch the ID of a PointRef.

      \return  Current point ID.
    */
    PointId pointId() const
        { return m_idx; }

    inline void getField(char *val, Dimension::Id d,
        Dimension::Type type) const;
    inline void setField(Dimension::Id dim,
        Dimension::Type type, const void *val);
    inline void toMetadata(MetadataNode node) const;
    inline MetadataNode toMetadata() const;

    /// Fill a buffer with point data specified by the dimension list.
    /// \param[in] dims  List of dimensions/types to retrieve.
    /// \param[in] idx   Index of point to get.
    /// \param[in] buf   Pointer to buffer to fill.
    void getPackedData(const DimTypeList& dims, char *buf) const
    {
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            getField(buf, di->m_id, di->m_type);
            buf += Dimension::size(di->m_type);
        }
    }

    /// Load the point buffer from memory whose arrangement is specified
    /// by the dimension list.
    /// \param[in] dims  Dimension/types of data in packed order
    /// \param[in] idx   Index of point to write.
    /// \param[in] buf   Packed data buffer.
    void setPackedData(const DimTypeList& dims, const char *buf)
    {
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            setField(di->m_id, di->m_type, (const void *)buf);
            buf += Dimension::size(di->m_type);
        }
    }

    bool compare(Dimension::Id dim, const PointRef& r) const
    {
        assert(m_container == r.m_container);
        return m_container->compare(dim, m_idx, r.m_idx);
    }

    static void swap(const PointRef& r1, const PointRef& r2)
    {
        assert(r1.m_container == r2.m_container);
        r1.m_container->swapItems(r1.m_idx, r2.m_idx);
    }

private:
    PointContainer *m_container;
    PointLayout *m_layout;
    PointId m_idx;
    bool m_tmp;
};

inline void swap(const PointRef& r1, const PointRef& r2)
{
    PointRef::swap(r1, r2);
}


/**
  Get the value of a dimension of a point.

  \param val  Pointer to buffer to which data should be written.  It must
    be large enough to contain data of the requested \type.
  \param d  Dimension to fetch.
  \param type  Type to which data should be converted when fetched.
*/
inline void PointRef::getField(char *val, Dimension::Id d,
    Dimension::Type type) const
{
    Everything e;

    switch (type)
    {
    case Dimension::Type::Float:
        e.f = getFieldAs<float>(d);
        break;
    case Dimension::Type::Double:
        e.d = getFieldAs<double>(d);
        break;
    case Dimension::Type::Signed8:
        e.s8 = getFieldAs<int8_t>(d);
        break;
    case Dimension::Type::Signed16:
        e.s16 = getFieldAs<int16_t>(d);
        break;
    case Dimension::Type::Signed32:
        e.s32 = getFieldAs<int32_t>(d);
        break;
    case Dimension::Type::Signed64:
        e.s64 = getFieldAs<int64_t>(d);
        break;
    case Dimension::Type::Unsigned8:
        e.u8 = getFieldAs<uint8_t>(d);
        break;
    case Dimension::Type::Unsigned16:
        e.u16 = getFieldAs<uint16_t>(d);
        break;
    case Dimension::Type::Unsigned32:
        e.u32 = getFieldAs<uint32_t>(d);
        break;
    case Dimension::Type::Unsigned64:
        e.u64 = getFieldAs<uint64_t>(d);
        break;
    case Dimension::Type::None:
        e.u64 = 0;
        break;
    }
    memcpy(val, &e, Dimension::size(type));
}


/**
  Set the value of a field in a point.

  \param dim  Dimension to set.
  \param type  PDAL type of data pointed to by \val
  \param val  Pointer to data to set.
*/
inline void PointRef::setField(Dimension::Id dim,
    Dimension::Type type, const void *val)
{
    Everything e;

    memcpy(&e, val, Dimension::size(type));
    switch (type)
    {
        case Dimension::Type::Float:
            setField(dim, e.f);
            break;
        case Dimension::Type::Double:
            setField(dim, e.d);
            break;
        case Dimension::Type::Signed8:
            setField(dim, e.s8);
            break;
        case Dimension::Type::Signed16:
            setField(dim, e.s16);
            break;
        case Dimension::Type::Signed32:
            setField(dim, e.s32);
            break;
        case Dimension::Type::Signed64:
            setField(dim, e.s64);
            break;
        case Dimension::Type::Unsigned8:
            setField(dim, e.u8);
            break;
        case Dimension::Type::Unsigned16:
            setField(dim, e.u16);
            break;
        case Dimension::Type::Unsigned32:
            setField(dim, e.u32);
            break;
        case Dimension::Type::Unsigned64:
            setField(dim, e.u64);
            break;
        case Dimension::Type::None:
            break;
    }
}

/**
  Convert a PointRef to metadata.

  \return  A node containing the PointRef as metadata.
*/
inline MetadataNode PointRef::toMetadata() const
{
    MetadataNode node;

    toMetadata(node);
    return node;
}

/**
  Add PointRef data to a root MetadataNode

  \param root  Root node to which point data should be added.
*/
inline void PointRef::toMetadata(MetadataNode root) const
{
    for (Dimension::Id id : m_layout->dims())
    {
        double v = getFieldAs<double>(id);
        root.add(m_layout->dimName(id), v);
    }
}

} // namespace pdal
