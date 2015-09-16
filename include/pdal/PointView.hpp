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

#include <pdal/util/Bounds.hpp>
#include <pdal/pdal_internal.hpp>
#include <pdal/PointLayout.hpp>
#include <pdal/PointTable.hpp>

#include <memory>
#include <queue>
#include <set>
#include <vector>
#include <deque>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4244)  // conversion from 'type1' to 'type2', possible loss of data
#endif

namespace pdal
{
namespace plang
{
    class BufferedInvocation;
}

struct PointViewLess;
class PointView;
class PointViewIter;

typedef std::shared_ptr<PointView> PointViewPtr;
typedef std::set<PointViewPtr, PointViewLess> PointViewSet;

class PDAL_DLL PointView
{
    friend class plang::BufferedInvocation;
    friend class PointRef;
    friend struct PointViewLess;
public:
    PointView(PointTableRef pointTable) : m_pointTable(pointTable),
        m_size(0), m_id(0)
    {
        static int lastId = 0;
        m_id = ++lastId;
    }

    virtual ~PointView()
    {}

    PointViewIter begin();
    PointViewIter end();

    int id() const
        { return m_id; }

    point_count_t size() const
        { return m_size; }

    bool empty() const
        { return m_size == 0; }

    inline void appendPoint(const PointView& buffer, PointId id);
    void append(const PointView& buf)
    {
        // We use size() instead of the index end because temp points
        // might have been placed at the end of the buffer.
        auto thisEnd = m_index.begin() + size();
        auto bufEnd = buf.m_index.begin() + buf.size();
        m_index.insert(thisEnd, buf.m_index.begin(), bufEnd);
        m_size += buf.size();
        clearTemps();
    }

    /// Return a new point view with the same point table as this
    /// point buffer.
    PointViewPtr makeNew() const
        { return PointViewPtr(new PointView(m_pointTable)); }

    template<class T>
    T getFieldAs(Dimension::Id::Enum dim, PointId pointIndex) const;

    inline void getField(char *pos, Dimension::Id::Enum d,
        Dimension::Type::Enum type, PointId id) const;

    template<typename T>
    void setField(Dimension::Id::Enum dim, PointId idx, T val);

    inline void setField(Dimension::Id::Enum dim, Dimension::Type::Enum type,
        PointId idx, const void *val);

    template <typename T>
    bool compare(Dimension::Id::Enum dim, PointId id1, PointId id2)
    {
        return (getFieldInternal<T>(dim, id1) < getFieldInternal<T>(dim, id2));
    }

    bool compare(Dimension::Id::Enum dim, PointId id1, PointId id2)
    {
        const Dimension::Detail *dd = m_pointTable.layout()->dimDetail(dim);

        switch (dd->type())
        {
            case Dimension::Type::Float:
                return compare<float>(dim, id1, id2);
                break;
            case Dimension::Type::Double:
                return compare<double>(dim, id1, id2);
                break;
            case Dimension::Type::Signed8:
                return compare<int8_t>(dim, id1, id2);
                break;
            case Dimension::Type::Signed16:
                return compare<int16_t>(dim, id1, id2);
                break;
            case Dimension::Type::Signed32:
                return compare<int32_t>(dim, id1, id2);
                break;
            case Dimension::Type::Signed64:
                return compare<int64_t>(dim, id1, id2);
                break;
            case Dimension::Type::Unsigned8:
                return compare<uint8_t>(dim, id1, id2);
                break;
            case Dimension::Type::Unsigned16:
                return compare<uint16_t>(dim, id1, id2);
                break;
            case Dimension::Type::Unsigned32:
                return compare<uint32_t>(dim, id1, id2);
                break;
            case Dimension::Type::Unsigned64:
                return compare<uint64_t>(dim, id1, id2);
                break;
            case Dimension::Type::None:
            default:
                return false;
                break;
        }
    }

    void getRawField(Dimension::Id::Enum dim, PointId idx, void *buf) const
    {
        getFieldInternal(dim, idx, buf);
    }

    /*! @return a cumulated bounds of all points in the PointView.
        \verbatim embed:rst
        .. note::

            This method requires that an `X`, `Y`, and `Z` dimension be
            available, and that it can be casted into a *double* data
            type using the :cpp:func:`pdal::Dimension::applyScaling`
            method. Otherwise, an exception will be thrown.
        \endverbatim
    */
    void calculateBounds(BOX2D& box) const;
    static void calculateBounds(const PointViewSet&, BOX2D& box);
    void calculateBounds(BOX3D& box) const;
    static void calculateBounds(const PointViewSet&, BOX3D& box);

    void dump(std::ostream& ostr) const;
    bool hasDim(Dimension::Id::Enum id) const
        { return m_pointTable.layout()->hasDim(id); }
    std::string dimName(Dimension::Id::Enum id) const
        { return m_pointTable.layout()->dimName(id); }
    Dimension::IdList dims() const
        { return m_pointTable.layout()->dims(); }
    std::size_t pointSize() const
        { return m_pointTable.layout()->pointSize(); }
    std::size_t dimSize(Dimension::Id::Enum id) const
        { return m_pointTable.layout()->dimSize(id); }
    Dimension::Type::Enum dimType(Dimension::Id::Enum id) const
     { return m_pointTable.layout()->dimType(id);}
    DimTypeList dimTypes() const
        { return m_pointTable.layout()->dimTypes(); }

    PointLayoutPtr layout() const { return m_pointTable.layout(); }

    /// Fill a buffer with point data specified by the dimension list.
    /// \param[in] dims  List of dimensions/types to retrieve.
    /// \param[in] idx   Index of point to get.
    /// \param[in] buf   Pointer to buffer to fill.
    void getPackedPoint(const DimTypeList& dims, PointId idx, char *buf) const
    {
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            getField(buf, di->m_id, di->m_type, idx);
            buf += Dimension::size(di->m_type);
        }
    }

    /// Load the point buffer from memory whose arrangement is specified
    /// by the dimension list.
    /// \param[in] dims  Dimension/types of data in packed order
    /// \param[in] idx   Index of point to write.
    /// \param[in] buf   Packed data buffer.
    void setPackedPoint(const DimTypeList& dims, PointId idx, const char *buf)
    {
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            setField(di->m_id, di->m_type, idx, (const void *)buf);
            buf += Dimension::size(di->m_type);
        }
    }


    /// Provides access to the memory storing the point data.  Though this
    /// function is public, other access methods are safer and preferred.
    char *getPoint(PointId id)
        { return m_pointTable.getPoint(m_index[id]); }

    // The standard idiom is swapping with a stack-created empty queue, but
    // that invokes the ctor and probably allocates.  We've probably only got
    // one or two things in our queue, so just pop until we're empty.
    void clearTemps()
    {
        while (!m_temps.empty())
            m_temps.pop();
    }

protected:
    PointTableRef m_pointTable;
    std::deque<PointId> m_index;
    // The index might be larger than the size to support temporary point
    // references.
    point_count_t m_size;
    int m_id;
    std::queue<PointId> m_temps;

private:
    template<typename T_IN, typename T_OUT>
    bool convertAndSet(Dimension::Id::Enum dim, PointId idx, T_IN in);

    inline void setFieldInternal(Dimension::Id::Enum dim, PointId pointIndex,
        const void *value);
    template<class T>
    T getFieldInternal(Dimension::Id::Enum dim, PointId pointIndex) const;
    inline void getFieldInternal(Dimension::Id::Enum dim, PointId pointIndex,
        void *value) const;
    inline PointId getTemp(PointId id);
    void freeTemp(PointId id)
        { m_temps.push(id); }

    // Awfulness to avoid exceptions in numeric cast.
    static bool m_ok;
};

struct PointViewLess
{
    bool operator () (const PointViewPtr& p1, const PointViewPtr& p2) const
        { return p1->m_id < p2->m_id; }
};

template <class T>
T PointView::getFieldInternal(Dimension::Id::Enum dim, PointId id) const
{
    T t;

    getFieldInternal(dim, id, &t);
    return t;
}

inline void PointView::getField(char *pos, Dimension::Id::Enum d,
    Dimension::Type::Enum type, PointId id) const
{
    Everything e;

    switch (type)
    {
    case Dimension::Type::Float:
        e.f = getFieldAs<float>(d, id);
        break;
    case Dimension::Type::Double:
        e.d = getFieldAs<double>(d, id);
        break;
    case Dimension::Type::Signed8:
        e.s8 = getFieldAs<int8_t>(d, id);
        break;
    case Dimension::Type::Signed16:
        e.s16 = getFieldAs<int16_t>(d, id);
        break;
    case Dimension::Type::Signed32:
        e.s32 = getFieldAs<int32_t>(d, id);
        break;
    case Dimension::Type::Signed64:
        e.s64 = getFieldAs<int64_t>(d, id);
        break;
    case Dimension::Type::Unsigned8:
        e.u8 = getFieldAs<uint8_t>(d, id);
        break;
    case Dimension::Type::Unsigned16:
        e.u16 = getFieldAs<uint16_t>(d, id);
        break;
    case Dimension::Type::Unsigned32:
        e.u32 = getFieldAs<uint32_t>(d, id);
        break;
    case Dimension::Type::Unsigned64:
        e.u64 = getFieldAs<uint64_t>(d, id);
        break;
    case Dimension::Type::None:
        break;
    }
    memcpy(pos, &e, Dimension::size(type));
}

inline void PointView::setField(Dimension::Id::Enum dim,
    Dimension::Type::Enum type, PointId idx, const void *val)
{
    Everything e;

    memcpy(&e, val, Dimension::size(type));
    switch (type)
    {
        case Dimension::Type::Float:
            setField(dim, idx, e.f);
            break;
        case Dimension::Type::Double:
            setField(dim, idx, e.d);
            break;
        case Dimension::Type::Signed8:
            setField(dim, idx, e.s8);
            break;
        case Dimension::Type::Signed16:
            setField(dim, idx, e.s16);
            break;
        case Dimension::Type::Signed32:
            setField(dim, idx, e.s32);
            break;
        case Dimension::Type::Signed64:
            setField(dim, idx, e.s64);
            break;
        case Dimension::Type::Unsigned8:
            setField(dim, idx, e.u8);
            break;
        case Dimension::Type::Unsigned16:
            setField(dim, idx, e.u16);
            break;
        case Dimension::Type::Unsigned32:
            setField(dim, idx, e.u32);
            break;
        case Dimension::Type::Unsigned64:
            setField(dim, idx, e.u64);
            break;
        case Dimension::Type::None:
            break;
    }
}

template <class T>
inline T PointView::getFieldAs(Dimension::Id::Enum dim,
    PointId pointIndex) const
{
    T retval;
    const Dimension::Detail *dd = m_pointTable.layout()->dimDetail(dim);
    double val;

    switch (dd->type())
    {
    case Dimension::Type::Float:
        val = getFieldInternal<float>(dim, pointIndex);
        break;
    case Dimension::Type::Double:
        val = getFieldInternal<double>(dim, pointIndex);
        break;
    case Dimension::Type::Signed8:
        val = getFieldInternal<int8_t>(dim, pointIndex);
        break;
    case Dimension::Type::Signed16:
        val = getFieldInternal<int16_t>(dim, pointIndex);
        break;
    case Dimension::Type::Signed32:
        val = getFieldInternal<int32_t>(dim, pointIndex);
        break;
    case Dimension::Type::Signed64:
        val = static_cast<double>(getFieldInternal<int64_t>(dim, pointIndex));
        break;
    case Dimension::Type::Unsigned8:
        val = getFieldInternal<uint8_t>(dim, pointIndex);
        break;
    case Dimension::Type::Unsigned16:
        val = getFieldInternal<uint16_t>(dim, pointIndex);
        break;
    case Dimension::Type::Unsigned32:
        val = getFieldInternal<uint32_t>(dim, pointIndex);
        break;
    case Dimension::Type::Unsigned64:
        val = static_cast<double>(getFieldInternal<uint64_t>(dim, pointIndex));
        break;
    case Dimension::Type::None:
    default:
        val = 0;
        break;
    }

    if (!Utils::numericCast(val, retval))
    {
        std::ostringstream oss;
        oss << "Unable to fetch data and convert as requested: ";
        oss << Dimension::name(dim) << ":" <<
            Dimension::interpretationName(dd->type()) <<
            "(" << (double)val << ") -> " << Utils::typeidName<T>();
        throw pdal_error(oss.str());
    }

    return retval;
}


template<typename T_IN, typename T_OUT>
bool PointView::convertAndSet(Dimension::Id::Enum dim, PointId idx, T_IN in)
{
    T_OUT out;

    bool success = Utils::numericCast(in, out);
    if (success)
        setFieldInternal(dim, idx, &out);
    return success;
}


template<typename T>
void PointView::setField(Dimension::Id::Enum dim, PointId idx, T val)
{
    const Dimension::Detail *dd = m_pointTable.layout()->dimDetail(dim);

    bool ok = true;
    switch (dd->type())
    {
    case Dimension::Type::Float:
        ok = convertAndSet<T, float>(dim, idx, val);
        break;
    case Dimension::Type::Double:
        ok = convertAndSet<T, double>(dim, idx, val);
        break;
    case Dimension::Type::Signed8:
        setFieldInternal(dim, idx, &val);
        break;
    case Dimension::Type::Signed16:
        ok = convertAndSet<T, int16_t>(dim, idx, val);
        break;
    case Dimension::Type::Signed32:
        ok = convertAndSet<T, int32_t>(dim, idx, val);
        break;
    case Dimension::Type::Signed64:
        ok = convertAndSet<T, int64_t>(dim, idx, val);
        break;
    case Dimension::Type::Unsigned8:
        setFieldInternal(dim, idx, &val);
        break;
    case Dimension::Type::Unsigned16:
        ok = convertAndSet<T, uint16_t>(dim, idx, val);
        break;
    case Dimension::Type::Unsigned32:
        ok = convertAndSet<T, uint32_t>(dim, idx, val);
        break;
    case Dimension::Type::Unsigned64:
        ok = convertAndSet<T, uint64_t>(dim, idx, val);
        break;
    case Dimension::Type::None:
        val = 0;
        break;
    }
    if (!ok)
    {
        std::ostringstream oss;
        oss << "Unable to set data and convert as requested: ";
        oss << Dimension::name(dim) << ":" << Utils::typeidName<T>() <<
            "(" << (double)val << ") -> " <<
            Dimension::interpretationName(dd->type());
        throw pdal_error(oss.str());
    }
}


inline void PointView::getFieldInternal(Dimension::Id::Enum dim,
    PointId id, void *buf) const
{
    m_pointTable.getField(m_pointTable.layout()->dimDetail(dim),
        m_index[id], buf);
}


inline void PointView::setFieldInternal(Dimension::Id::Enum dim,
    PointId id, const void *value)
{
    PointId rawId = 0;
    if (id == size())
    {
        rawId = m_pointTable.addPoint();
        m_index.push_back(rawId);
        m_size++;
        assert(m_temps.empty());
    }
    else if (id > size())
    {
        std::cerr << "Point index must increment.\n";
        //error - throw?
        return;
    }
    else
    {
        rawId = m_index[id];
    }
    m_pointTable.setField(m_pointTable.layout()->dimDetail(dim),
        rawId, value);
}


inline void PointView::appendPoint(const PointView& buffer, PointId id)
{
    // Invalid 'id' is a programmer error.
    PointId rawId = buffer.m_index[id];
    m_index.push_back(rawId);
    m_size++;
    assert(m_temps.empty());
}


// Make a temporary copy of a point by adding an entry to the index.
inline PointId PointView::getTemp(PointId id)
{
    PointId newid;
    if (m_temps.size())
    {
        newid = m_temps.front();
        m_temps.pop();
        m_index[newid] = m_index[id];
    }
    else
    {
        newid = (PointId)m_index.size();
        m_index.push_back(m_index[id]);
    }
    return newid;
}

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const PointView&);

} // namespace pdal
