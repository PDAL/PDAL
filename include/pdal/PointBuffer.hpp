/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/pdal_internal.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/PointContext.hpp>
#include <pdal/Schema.hpp>

#include <vector>
#include <set>

namespace pdal
{
    namespace pointbuffer
    {
        typedef boost::uint64_t PointBufferByteSize;
    } // pointbuffer

/*! 
    \verbatim embed:rst
    .. note::

        The arrangement of PointBuffer's bytes might either be
        point-interleaved or dimension-interleaved, with point-interleave being
        the default organization.  If you are directly modifying a PointBuffer's
        bytes, you must respect the :cpp:class:`pdal::pointbuffer::Orientation`. 
    \endverbatim
*/    
class PDAL_DLL PointBuffer
{
public:
    PointBuffer();
    PointBuffer(PointContext context) :
        m_bounds(Bounds<double>::getDefaultSpatialExtent()), m_context(context)
    {}

    /** @name Attribute access
    */
    /*! @return the pdal::Bounds instance associated with this pdal::PointBuffer.
        \verbatim embed:rst
        .. note::

            It is not a requirement that stages keep the pdal::Bounds instance
            up-to-date when operating on the PointBuffer.
        \endverbatim
    */
    const Bounds<double>& getSpatialBounds() const
        { return m_bounds; }

    /// sets the pdal::Bounds instance for this pdal::PointBuffer
    /// @param bounds bounds instance to set.
    void setSpatialBounds(const Bounds<double>& bounds)
        { m_bounds = bounds; }

    point_count_t size() const
        { return m_index.size(); }

    inline void appendPoint(PointBuffer& buffer, PointId id);

    /// A const reference to the internally copied pdal::Schema instance that
    /// was given at construction time.
    const Schema& getSchema() const
        { return *(m_context.schema()); }

    /// Get the buffer's point context.
    PointContext context() const
        { return m_context; }

    /** @name Point data access
    */
    /*! fetch the value T for a given :cpp:class:`pdal::Dimension` dim at
        pointIndex `i`.
        \param dim  The dimension to select
        \param pointIndex the point index of the PointBuffer to select.
        \verbatim embed:rst
        .. warning::

            If the data type of T is not the same as described in
            :cpp:class:`pdal::Dimension`, the data value will be cast into
            the appropriate type. In some situations this may not be what
            you want. In situations where the T is smaller than the
            datatype given by `dim`, the return value T will simply be
            saturated.
        \endverbatim
    */
    template<class T>
    T getField(Dimension const& dim, PointId pointIndex) const;

    /*! fetch the value T for a given :cpp:class:`pdal::Dimension` dim at
        pointIndex `i`.
        \param dim the dimension to select.
        \param pointIndex the point index of the PointBuffer to select.
        \param applyScaling whether or not to apply the dimension's scale and
               offset prior to returning the value.
        \verbatim embed:rst
        .. note::

            The method will attempt to cast :cpp:class:`pdal::Dimension` to
            the requested data type T. If a bad cast is detected, pdal_error
            is thrown.
        \endverbatim
    */
    template<class T>
    T getFieldAs(Dimension const& dim, PointId pointIndex,
        bool applyScaling = true) const;

    /*! set the value T for a given  :cpp:class:`pdal::Dimension` dim
        at pointIndex i.
        \param dim  The dimension to select.
        \param pointIndex the point index of the PointBuffer to select.
        \param value the T value to set
        \verbatim embed:rst
        .. warning::

            If the data type of T is not the same as described in
            :cpp:class:`pdal::Dimension`, the data value will be cast 
            into the appropriate type. In some situations this may not
            be what you want. In situations where the T is smaller than
            the datatype given by `dim`, the return value T will simply be
            saturated.
        \endverbatim
    */
    template<typename T>
    void setField(Dimension const& dim, PointId idx, T val);

    void setRawField(Dimension const& dim, PointId idx, const void *val)
    {
        setFieldInternal(dim, idx, val);
    }

    void getRawField(const Dimension& dim, PointId idx, void *buf) const
    {
        getFieldInternal(dim, idx, buf);
    }

    void setFieldUnscaled(pdal::Dimension const& dim, PointId idx,
        double val)
    {
        setField(dim, idx, dim.removeScaling(val));
    }

    /** @name Serialization
    */
    /*! returns a boost::property_tree containing the point records, which is
        useful for dumping and such.
        \verbatim embed:rst
        ::

            0:
                X: 1.00
                Y: 2.00
                Z: 3.00
            1:
                X: 1.00
                Y: 2.00
                Z: 3.00

        .. note::

            If you want to print out details about the fields, e.g. the byte
            offset or the datatype, use the :cpp:func:`pdal::Schema::getPtree()`
            dumper.
        \endverbatim
    */
    boost::property_tree::ptree toPTree() const;
    std::ostream& toRST(std::ostream& os) const;

    /*! @return a cumulated bounds of all points in the PointBuffer.
        \verbatim embed:rst
        .. note::

            This method requires that an `X`, `Y`, and `Z` dimension be 
            available, and that it can be casted into a *double* data 
            type using the :cpp:func:`pdal::Dimension::applyScaling` 
            method. Otherwise, an exception will be thrown.
        \endverbatim
    */    
    pdal::Bounds<double> calculateBounds(bool bis3d=true) const;

protected:
    Bounds<double> m_bounds;
    PointContext m_context;
    std::vector<PointId> m_index;

private:
    template<typename T_IN, typename T_OUT>
    void convertAndSet(pdal::Dimension const& dim, PointId idx, T_IN in);

    inline void setFieldInternal(Dimension const& dim, PointId pointIndex,
        const void *value);
    inline void getFieldInternal(Dimension const& dim, PointId pointIndex,
        void *value) const;
};


template <class T>
T PointBuffer::getField(pdal::Dimension const& dim, PointId id) const
{
    T t;

    getFieldInternal(dim, id, &t);
    return t;
}


template <class T>
inline T PointBuffer::getFieldAs(pdal::Dimension const& dim,
    PointId pointIndex, bool applyScaling) const
{
    T retval;
    boost::uint32_t size = dim.getByteSize();
    double val(0.0);
#ifdef PDAL_COMPILER_MSVC
// warning C4127: conditional expression is constant
#pragma warning(push)
#pragma warning(disable:4127 4244)
#endif
    switch (dim.getInterpretation())
    {
        case dimension::Float:
            switch (size)
            {
                case 4:
                    val = getField<float>(dim, pointIndex);
                    break;
                case 8:
                    val = getField<double>(dim, pointIndex);
                    break;
            }
            break;

        case dimension::SignedInteger:
            switch (size)
            {
              case 1:
                  val = getField<boost::int8_t>(dim, pointIndex);
                  break;
              case 2:
                  val = getField<boost::int16_t>(dim, pointIndex);
                  break;
              case 4:
                  val = getField<boost::int32_t>(dim, pointIndex);
                  break;
              case 8:
                  val = getField<boost::int64_t>(dim, pointIndex);
                  break;
            }
            break;

        case dimension::UnsignedInteger:
            switch (size)
            {
                case 1:
                    val = getField<boost::uint8_t>(dim, pointIndex);
                    break;
                case 2:
                    val = getField<boost::uint16_t>(dim, pointIndex);
                    break;
                case 4:
                    val = getField<boost::uint32_t>(dim, pointIndex);
                    break;
                case 8:
                    val = getField<boost::uint64_t>(dim, pointIndex);
                    break;
            }
            break;

        case dimension::RawByte:
        case dimension::Pointer:
        case dimension::Undefined:
            throw pdal_error("Dimension data type unable to be retrieved "
                "in getFieldAs");
    }

    if (applyScaling)
        val = dim.applyScaling(val);

    try
    {

    	if (std::is_integral<T>::value == true )
            retval = boost::numeric_cast<T>(lround(val));
        else
            retval = boost::numeric_cast<T>(val);
    }
    catch (boost::numeric::bad_numeric_cast& )
    {
        std::ostringstream oss;
        oss << "Unable to fetch data and convert as requested: ";
        oss << dim.getName() << ":" << dim.getInterpretationName() <<
            "(" << (double)val << ") -> " << Utils::typeidName<T>();
        throw pdal_error(oss.str());
    }
    return retval;
#ifdef PDAL_COMPILER_MSVC
// warning C4127: conditional expression is constant
#pragma warning(pop)
#endif	
}


template<typename T_IN, typename T_OUT>
void PointBuffer::convertAndSet(pdal::Dimension const& dim, PointId idx,
    T_IN in)
{
    T_OUT out;
	
#ifdef PDAL_COMPILER_MSVC
// warning C4127: conditional expression is constant
#pragma warning(push)
#pragma warning(disable:4127)
#endif
    if (std::is_integral<T_OUT>::value == true)
        out = boost::numeric_cast<T_OUT>(lround(in));
    else
        out = boost::numeric_cast<T_OUT>(in);

#ifdef PDAL_COMPILER_MSVC
// warning C4127: conditional expression is constant
#pragma warning(pop)
#endif

    setFieldInternal(dim, idx, (void *)&out);
}


template<typename T>
void PointBuffer::setField(pdal::Dimension const& dim, PointId idx, T val)
{
    uint32_t size = dim.getByteSize();
    try {
        switch (dim.getInterpretation())
        {
            case dimension::Float:
                switch (size)
                {
                    case 4:
                        convertAndSet<T, float>(dim, idx, val);
                        break;
                    case 8:
                        convertAndSet<T, double>(dim, idx, val);
                        break;
                }
                break;

            case dimension::SignedInteger:
                switch (size)
                {
                    case 1:
                        convertAndSet<T, int8_t>(dim, idx, val);
                        break;
                    case 2:
                        convertAndSet<T, int16_t>(dim, idx, val);
                        break;
                    case 4:
                        convertAndSet<T, int32_t>(dim, idx, val);
                        break;
                    case 8:
                        convertAndSet<T, int64_t>(dim, idx, val);
                        break;
                }
                break;

            case dimension::UnsignedInteger:
                switch (size)
                {
                    case 1:
                        convertAndSet<T, uint8_t>(dim, idx, val);
                        break;
                    case 2:
                        convertAndSet<T, uint16_t>(dim, idx, val);
                        break;
                    case 4:
                        convertAndSet<T, uint32_t>(dim, idx, val);
                        break;
                    case 8:
                        convertAndSet<T, uint64_t>(dim, idx, val);
                        break;
                }
                break;

            case dimension::RawByte:
                setFieldInternal(dim, idx, &val);
                break;

            case dimension::Pointer:
            case dimension::Undefined:
                throw pdal_error("Dimension data type unable to be set.");
        }
    }
    catch (boost::numeric::bad_numeric_cast& )
    {
        std::ostringstream oss;
        oss << "Unable to set data and convert as requested: ";
        oss << dim.getName() << ":" << Utils::typeidName<T>() <<
            "(" << (double)val << ") -> " << dim.getInterpretationName();
        throw pdal_error(oss.str());
    }
}


inline void PointBuffer::getFieldInternal(pdal::Dimension const& dim,
    PointId id, void *buf) const
{
    m_context.rawPtBuf()->getField(dim, m_index[id], buf);
}


inline void PointBuffer::setFieldInternal(pdal::Dimension const& dim,
    PointId id, const void *value)
{
    PointId rawId = 0;
    if (id == m_index.size())
    {
        rawId = m_context.rawPtBuf()->addPoint();
        m_index.resize(id + 1);
        m_index[id] = rawId;
    }
    else if (id > m_index.size())
    {
        std::cerr << "Point index must increment.\n";
        //error - throw?
        return;
    }
    else
    {
        rawId = m_index[id];
    }

    m_context.rawPtBuf()->setField(dim, rawId, value);
}


inline void PointBuffer::appendPoint(PointBuffer& buffer, PointId id)
{
    // FIXME: hobu -- what happens if id is out of range of m_index
    // or m_index isn't ordered?
    PointId rawId = buffer.m_index[id];
    point_count_t newid = m_index.size();
    m_index.resize(newid + 1);
    m_index[newid] = rawId;
}

typedef std::shared_ptr<PointBuffer> PointBufferPtr;
typedef std::set<PointBufferPtr> PointBufferSet;

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const PointBuffer&);

} // namespace pdal

