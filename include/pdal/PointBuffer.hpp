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
    PointBuffer(const Schema&, boost::uint32_t capacity);
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
    template<class T> T getField(std::size_t pointIndex, boost::int32_t fieldIndex) const;
    template<class T> T getRawField(std::size_t pointIndex, std::size_t pointBytePosition) const;
    template<class T> void setField(std::size_t pointIndex, boost::int32_t fieldIndex, T value);
    void setFieldData(std::size_t pointIndex, boost::int32_t fieldIndex, const boost::uint8_t* data);
    
    // bulk copy all the fields from the given point into this object
    // NOTE: this is only legal if the src and dest schemas are exactly the same
    // (later, this will be implemented properly, to handle the general cases slowly and the best case quickly)
    inline void copyPointFast(std::size_t destPointIndex, std::size_t srcPointIndex, const PointBuffer& srcPointBuffer)
    {
        const boost::uint8_t* src = srcPointBuffer.getData(srcPointIndex);
        boost::uint8_t* dest = getData(destPointIndex);
        const std::size_t len = getSchema().getByteSize();

        memcpy(dest, src, len);

        assert(m_numPoints <= m_capacity);

        return;
    }
    
    // same as above, but copies N points
    inline void copyPointsFast(std::size_t destPointIndex, std::size_t srcPointIndex, const PointBuffer& srcPointBuffer, std::size_t numPoints)
    {
        const boost::uint8_t* src = srcPointBuffer.getData(srcPointIndex);
        boost::uint8_t* dest = getData(destPointIndex);
        const std::size_t len = getSchema().getByteSize();

        memcpy(dest, src, len * numPoints);

        assert(m_numPoints <= m_capacity);

        return;
    }
    
    inline boost::uint64_t getBufferByteLength() const
    {
        return m_pointSize*m_numPoints;
    }
    inline boost::uint64_t getBufferByteCapacity() const
    {
        return m_pointSize*m_capacity;
    }


    // access to the raw memory
    inline boost::uint8_t* getData(std::size_t pointIndex) const
    {
        return m_data.get() + m_pointSize * pointIndex;
    }

    inline boost::uint8_t* getData(std::size_t pointIndex)
    {
        return m_data.get() + m_pointSize * pointIndex;
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
    std::size_t m_pointSize;
    boost::uint32_t m_numPoints;
    boost::uint32_t m_capacity;    
    Bounds<double> m_bounds;
};


template <class T>
inline void PointBuffer::setField(std::size_t pointIndex, boost::int32_t fieldIndex, T value)
{
    if (fieldIndex == -1)
    {
        // this is a little harsh, but we'll keep it for now as we shake things out
        throw pdal_error("filedIndex is not valid at this point of access");
    }

    const Dimension& dim = m_schema.getDimension(fieldIndex);

    std::size_t offset = (pointIndex * m_pointSize) + dim.getByteOffset();
    assert(offset + sizeof(T) <= m_pointSize * m_capacity);
    boost::uint8_t* p = m_data.get() + offset;

    *(T*)p = value;
}

inline void PointBuffer::setFieldData(std::size_t pointIndex, boost::int32_t fieldIndex, const boost::uint8_t* data)
{
    if (fieldIndex == -1)
    {
        // this is a little harsh, but we'll keep it for now as we shake things out
        throw pdal_error("filedIndex is not valid at this point of access");
    }
    
    const Dimension& dim = m_schema.getDimension(fieldIndex);

    std::size_t offset = (pointIndex * m_pointSize) + dim.getByteOffset();
    std::size_t size = dim.getDataTypeSize(dim.getDataType());
    // std::cout << "copying field " << d.getFieldName() << " with index" << fieldIndex << " of size " << size << " at offset " << offset << std::endl;
    // assert(offset + sizeof(T) <= m_pointSize * m_capacity);
    boost::uint8_t* p = m_data.get() + offset;
    
    memcpy(p, data, size);
}


template <class T>
inline T PointBuffer::getField(std::size_t pointIndex, boost::int32_t fieldIndex) const
{
    if (fieldIndex == -1)
    {
        // this is a little harsh, but we'll keep it for now as we shake things out
        throw pdal_error("filedIndex is not valid at this point of access");
    }
        
    const Dimension& dim = m_schema.getDimension(fieldIndex);

    std::size_t offset = (pointIndex * m_pointSize) + dim.getByteOffset();
    assert(offset + sizeof(T) <= m_pointSize * m_capacity);
    boost::uint8_t* p = m_data.get() + offset;

    return *(T*)p;
}

template <class T>
inline T PointBuffer::getRawField(std::size_t pointIndex, std::size_t pointBytePosition) const
{
    std::size_t offset = (pointIndex * m_pointSize) + pointBytePosition;
    boost::uint8_t* p = m_data.get() + offset;

    return *(T*)p;
}

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const PointBuffer&);


} // namespace pdal

#endif
