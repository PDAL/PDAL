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

#ifndef INCLUDED_POINTBUFFER_HPP
#define INCLUDED_POINTBUFFER_HPP


#include <boost/scoped_array.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/type_traits.hpp>
#include <boost/algorithm/string.hpp>

#include <pdal/pdal_internal.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/Schema.hpp>

namespace pdal
{

// a PointBuffer object is just an untyped array of N bytes,
// where N is (the size of the given Schema * the number of points)
//
// That is, a PointBuffer represents the underlying data for one or more points.
//
// A PointBuffer object has an associated Schema object.
//
// Many of the methods take a first parameter "index", to specify which point in the
// collection is to be operated upon.  The point index is a uint32; you can't read
// more than 4 billion points at a time.
class PDAL_DLL PointBuffer

{
    
public:
    
    // note that when we make a PointBuffer object all the fields are initialized to inactive,
    // regardless of what the passed-in schema says -- this is because the field object
    // represents the state within the owning object, which in this case is a completely
    // empty buffer (similarly, all the points in the buffer are marked "invalid")
    PointBuffer(const Schema&, boost::uint32_t capacity=65536);
    PointBuffer(const PointBuffer&); 
    PointBuffer& operator=(const PointBuffer&); 

    ~PointBuffer();

    const Bounds<double>& getSpatialBounds() const;
    void setSpatialBounds(const Bounds<double>& bounds);

    // This is the number of points in this buffer that actually have valid data.  This number
    // will be less than or equal to the getCapacity() value.
    boost::uint32_t getNumPoints() const;
    
    inline void setNumPoints(boost::uint32_t v) { assert(v <= m_capacity);m_numPoints = v; } 
 
    // This is the number of points that this buffer is allocated to be able to store.
    // This is a fixed constant, set at ctor time by the person constructing the buffer.
    inline boost::uint32_t getCapacity() const { return m_capacity; }

    const Schema& getSchema() const
    {
        return m_schema;
    }

    // accessors to a particular field of a particular point in this buffer
    template<class T> T getField(Dimension const& dim, std::size_t pointIndex) const;    
    template<class T> T getRawField(std::size_t pointIndex, std::size_t pointBytePosition) const;
    template<class T> void setField(Dimension const& dim, std::size_t pointIndex, T value);
    template<class T> T convertDimension(pdal::Dimension const& dim, void* bytes) const;
    template<typename Target, typename Source> static Target saturation_cast(Source const& src);
    // bulk copy all the fields from the given point into this object
    // NOTE: this is only legal if the src and dest schemas are exactly the same
    // (later, this will be implemented properly, to handle the general cases slowly and the best case quickly)
    inline void copyPointFast(std::size_t destPointIndex, std::size_t srcPointIndex, const PointBuffer& srcPointBuffer)
    {
        const boost::uint8_t* src = srcPointBuffer.getData(srcPointIndex);
        boost::uint8_t* dest = getData(destPointIndex);

        memcpy(dest, src, m_byteSize);

        assert(m_numPoints <= m_capacity);

        return;
    }
    
    // same as above, but copies N points
    inline void copyPointsFast(std::size_t destPointIndex, std::size_t srcPointIndex, const PointBuffer& srcPointBuffer, std::size_t numPoints)
    {
        const boost::uint8_t* src = srcPointBuffer.getData(srcPointIndex);
        boost::uint8_t* dest = getData(destPointIndex);

        memcpy(dest, src, m_byteSize * numPoints);

        assert(m_numPoints <= m_capacity);

        return;
    }
    
    inline boost::uint64_t getBufferByteLength() const
    {
        return m_byteSize * m_numPoints;
    }
    inline boost::uint64_t getBufferByteCapacity() const
    {
        return m_schema.getByteSize() * m_capacity;
    }


    // access to the raw memory
    inline boost::uint8_t* getData(std::size_t pointIndex) const
    {
        return m_data.get() + m_schema.getByteSize() * pointIndex;
    }

    inline boost::uint8_t* getData(std::size_t pointIndex)
    {
        return m_data.get() + m_schema.getByteSize() * pointIndex;
    }
    
    // copy in raw data
    void setData(boost::uint8_t* data, std::size_t pointIndex);
    
    void setAllData(boost::uint8_t* data, boost::uint32_t byteCount);
    void setDataStride(boost::uint8_t* data, std::size_t index, boost::uint32_t byteCount);

    // access to the raw memory
    void getData(boost::uint8_t** data, std::size_t* array_size) const;

    // returns a ptree containing the point records, useful for dumping
    // and such
    //
    // The tree will look like this:
    //
    //    0:
    //        X: 1.00
    //        Y: 2.00
    //        Z: 3.00
    //    1:
    //        X: 1.00
    //        Y: 2.00
    //        Z: 3.00
    //
    // If you want to print out details about the fields, e.g. the byte 
    // offset or the datatype, use the Schema's ptree dumper.
    // 
    boost::property_tree::ptree toPTree() const;

private:
    Schema m_schema;
    boost::scoped_array<boost::uint8_t> m_data;
    boost::uint32_t m_numPoints;
    boost::uint32_t m_capacity;    
    Bounds<double> m_bounds;
    schema::size_type m_byteSize;
};

template<typename Target, typename Source>
inline Target PointBuffer::saturation_cast(Source const& src) {

    try {
        return boost::numeric_cast<Target>(src);
    }
    catch (boost::numeric::negative_overflow const&) {
        return std::numeric_limits<Target>::min();
    }
    catch (boost::numeric::positive_overflow const&) {
        return std::numeric_limits<Target>::max();
    }

}

template <class T>
inline void PointBuffer::setField(pdal::Dimension const& dim, std::size_t pointIndex, T value)
{
    if (dim.getPosition() == -1)
    {
        // this is a little harsh, but we'll keep it for now as we shake things out
        throw buffer_error("This dimension has no identified position in a schema. Use the setRawField method to access an arbitrary byte position.");
    }

    std::size_t offset = (pointIndex * m_byteSize ) + dim.getByteOffset();
    assert(offset + sizeof(T) <= m_byteSize * m_capacity);
    boost::uint8_t* p = m_data.get() + offset;

    if (sizeof(T) == dim.getByteSize())
    {
        // Winner, winner, chicken dinner. We're not going to try to 
        // do anything magical. It's up to you to get the interpretation right.
        *(T*)(void*)p = value;
        return;
    }
    
    T output(0);
    output = boost::lexical_cast<T>(value);
    *(T*)(void*)p = output;


}

template <class T>
inline T PointBuffer::convertDimension(pdal::Dimension const& dim, void* p) const

{

    T output(0);

    float flt(0.0);
    double dbl(0.0);
    boost::int8_t i8(0);
    boost::uint8_t u8(0);
    boost::int16_t i16(0);
    boost::uint16_t u16(0);
    boost::int32_t i32(0);
    boost::uint32_t u32(0);
    boost::int64_t i64(0);
    boost::uint64_t u64(0);
    
    switch (dim.getInterpretation())
    {
        case dimension::SignedByte:
            i8 = *(boost::int8_t*)(void*)p;
            output = saturation_cast<T, int8_t>(i8);
            break;
        case dimension::UnsignedByte:
            u8 = *(boost::uint8_t*)(void*)p;
            output = saturation_cast<T, uint8_t>(u8);
            break;

        case dimension::SignedInteger:
            if (dim.getByteSize() == 1 )
            {
                i8 = *(boost::int8_t*)(void*)p;
                output = saturation_cast<T, int8_t>(i8);
            } else if (dim.getByteSize() == 2) 
            {
                i16 = *(boost::int16_t*)(void*)p;
                output = saturation_cast<T, int16_t>(i16);
                
            } else if (dim.getByteSize() == 4)
            {
                i32 = *(boost::int32_t*)(void*)p;
                output = saturation_cast<T, int32_t>(i32);
            } else if (dim.getByteSize() == 8)
            {
                i64 = *(boost::int64_t*)(void*)p;
                output = saturation_cast<T, int64_t>(i64);
            } else
            {
                throw buffer_error("getField::Unhandled datatype size for SignedInteger");
            }
            break;
        case dimension::UnsignedInteger:
            if (dim.getByteSize() == 1 )
            {
                u8 = *(boost::uint8_t*)(void*)p;
                output = saturation_cast<T, uint8_t>(u8);
            } else if (dim.getByteSize() == 2) 
            {
                u16 = *(boost::uint16_t*)(void*)p;
                output = saturation_cast<T, uint16_t>(u16);
                
            } else if (dim.getByteSize() == 4)
            {
                u32 = *(boost::uint32_t*)(void*)p;
                output = saturation_cast<T, uint32_t>(u32);
            } else if (dim.getByteSize() == 8)
            {
                u64 = *(boost::uint64_t*)(void*)p;
                output = saturation_cast<T, uint64_t>(u64);
            } else
            {
                throw buffer_error("getField::Unhandled datatype size for UnsignedInteger");
            }
        
            break;
        case dimension::Float:
            if (dim.getByteSize() == 4)
            {
                flt = *(float*)(void*)p;
                output = saturation_cast<T, float>(flt);
            } else if (dim.getByteSize() == 8)
            {
                dbl = *(double*)(void*)p;
                output = saturation_cast<T, double>(dbl);
            } else
            {
                throw buffer_error("getField::Unhandled datatype size for Float");
            }
            break;
        
        case dimension::Pointer:
            break;
        default:
            throw buffer_error("Undefined interpretation for getField");
    }

    return output;
    
}

template <class T>
inline T PointBuffer::getField(pdal::Dimension const& dim, std::size_t pointIndex) const
{
    if (dim.getPosition() == -1)
    {
        // this is a little harsh, but we'll keep it for now as we shake things out
        throw buffer_error("This dimension has no identified position in a schema. Use the getRawField method to access an arbitrary byte position.");
    }
        
    std::size_t offset = (pointIndex * m_schema.getByteSize()) + dim.getByteOffset();
    
    if (offset + sizeof(T) > m_byteSize * m_capacity)
    {
        std::ostringstream oss;
        oss << "Offset for given dimension is off the end of the buffer!";
        throw buffer_error(oss.str());
    }
    
    assert(offset + sizeof(T) <= m_byteSize * m_capacity);
    boost::uint8_t const* p = m_data.get() + offset;
    
#if 0
    // The user could be asking for data from a floating point dimension 
    // as an integer. In that case, simply returning a casted int from those 
    // bytes is not the number we want. We don't want to test *every* dimension 
    // combination either, as this code is definitely in the critical path. 
    // For now, we'll only test if the getInterpretation == dimension::Float, 
    // which shouldn't be the most common dimension-fetching scenario.
    if (sizeof(T) == dim.getByteSize() )
    {
        if (dim.getInterpretation() == dimension::Float)
        {
            if (boost::iequals(typeid(T).name(), "i"))
            {
                if (dim.getByteSize() == 4)
                {
                    return boost::numeric_cast<int>(*( float const*)p);
                    
                }
                else
                    return boost::numeric_cast<int>(*( double const*)p);
            }
        }

        return *(T const*)( void const*)p;
    }
#endif

    return convertDimension<T>(dim, (void *)p);

    
}

template <class T>
inline T PointBuffer::getRawField(std::size_t pointIndex, std::size_t pointBytePosition) const
{
    std::size_t offset = (pointIndex * m_byteSize ) + pointBytePosition;
    boost::uint8_t* p = m_data.get() + offset;

    return *(T*)(void*)p;
}

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const PointBuffer&);


} // namespace pdal

#endif
