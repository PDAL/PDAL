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
#include <pdal/Metadata.hpp>

#include <boost/optional.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/functional/hash.hpp>


namespace pdal
{

namespace pointbuffer {

    struct name{};
    struct index{};
    struct uid{};

    typedef boost::multi_index::multi_index_container<
      Metadata,
      boost::multi_index::indexed_by<

        boost::multi_index::random_access<boost::multi_index::tag<index> >,
        // sort by less<string> on GetName
        boost::multi_index::hashed_non_unique<boost::multi_index::tag<name>, boost::multi_index::const_mem_fun<Metadata,std::string const&,&Metadata::getName> >,
        boost::multi_index::hashed_non_unique<boost::multi_index::tag<uid>, boost::multi_index::const_mem_fun<Metadata,metadata::id const&,&Metadata::getUUID> >
          >
    > MetadataMap;

    typedef MetadataMap::index<name>::type index_by_name;
    typedef MetadataMap::index<index>::type index_by_index;
    typedef MetadataMap::index<uid>::type index_by_uid;

}

/// A PointBuffer is the object that is passed through pdal::Stage instances 
/// to form a pipeline. A PointBuffer is composed of a pdal::Schema that determines 
/// the layout of the data contained within, along with a dictionary of pdal::Metadata 
/// entries. The capacity of a PointBuffer is determined by the number of points 
/// it contains, and the number of points possible in a PointBuffer is limited to 
/// std::numeric_limits<boost::uint32_t>::max().  Underneath the covers, a PointBuffer 
/// is simply the composed array of bytes described in the pdal::Schema. You can 
/// operate on the raw bytes if you need to, but PointBuffer provides a number of 
/// convienence methods to make things easier.
class PDAL_DLL PointBuffer
{
public:

/** @name Constructors
*/  
    /*! Base constructor for pdal::PointBuffer.
        \param schema pdal::Schema instance to use to describe the layout. It is copied.
        \param capacity size of the pdal::PointBuffer in number of points.
        \verbatim embed:rst 
        .. note::
            
            All fields are initialized to inactive regardless of what the
            passed-in pdal::Schema says.
        \endverbatim
    */
    PointBuffer(const Schema& schema, boost::uint32_t capacity=65536);
    
    /// Copy constructor. The data array is simply memcpy'd.
    PointBuffer(const PointBuffer&); 
    
    /// Assignment constructor.
    PointBuffer& operator=(const PointBuffer&); 

    /// Destructor. 
    ~PointBuffer() {};

/** @name Attribute access
*/ 
    /*! returns the pdal::Bounds instance associated with this pdal::PointBuffer. 
        \verbatim embed:rst 
        .. note::
            
            It is not a requirement that stages keep the pdal::Bounds instance 
            up-to-date when operating on the PointBuffer.
        \endverbatim
    */
    const Bounds<double>& getSpatialBounds() const;
    
    /// sets the pdal::Bounds instance for this pdal::PointBuffer
    /// @param bounds bounds instance to set.
    void setSpatialBounds(const Bounds<double>& bounds);

    /// returns the number of points that are set for the PointBuffer. It is 
    /// an arbitrary number that must be <= getCapacity() is the number of 
    /// active points for the PointBuffer
    inline boost::uint32_t getNumPoints() const { return m_numPoints; }
    
    /// sets the number of active points for the PointBuffer.
    /// @param v number of points to set.
    inline void setNumPoints(boost::uint32_t v) { assert(v <= m_capacity);m_numPoints = v; } 

    /*! returns the maximum number of points the PointBuffer can hold. 
        \verbatim embed:rst 
        .. note::
            
            This value is given at construction time, and it in conjunction 
            with the given pdal::Schema determine the size of the raw byte 
            buffer that contains the point data. It cannot be changed.
        \endverbatim
    */ 
    inline boost::uint32_t getCapacity() const { return m_capacity; }

    /// A const reference to the internally copied pdal::Schema instance that 
    /// was given at construction time.
    const Schema& getSchema() const
    {
        return m_schema;
    }

    /// returns the size of the currently filled raw byte array
    /// Equivalent to getNumPoints() * getSchema() * getByteSize().    
    inline boost::uint64_t getBufferByteLength() const
    {
        return m_byteSize * m_numPoints;
    }
    
    /// returns the size of the theoretically filled raw byte array.  
    /// Equivalent to getCapacity() * getSchema() * getByteSize().
    inline boost::uint64_t getBufferByteCapacity() const
    {
        return m_byteSize * m_capacity;
    }

/** @name Point data access
*/ 
    /*! fetch the value T for a given pdal::Dimension dim at pointIndex i. 
        \param dim pdal::Dimension instance describing the dimension to select
        \param pointIndex the point index of the PointBuffer to select.
        \verbatim embed:rst 
        .. warning::
            
            If the data type of T is not the same as described in pdal::Dimension,
            the data value will be casted into the appropriate type. In some 
            situations this may not be what you want. In situations where the T 
            is smaller than the datatype given by `dim`, the return value T 
            will simply be saturated.
        \endverbatim
    */
    template<class T> T getField(Dimension const& dim, std::size_t pointIndex) const;    

    /*! set the value T for a given pdal::Dimension dim at pointIndex i. 
        \param dim pdal::Dimension instance describing the dimension to select
        \param pointIndex the point index of the PointBuffer to select.
        \param value the T value to set
        \verbatim embed:rst 
        .. warning::
            
            If the data type of T is not the same as described in pdal::Dimension,
            the data value will be casted into the appropriate type. In some 
            situations this may not be what you want. In situations where the T 
            is smaller than the datatype given by `dim`, the return value T 
            will simply be saturated.
        \endverbatim
    */   
    template<class T> void setField(Dimension const& dim, std::size_t pointIndex, T value);

    /*! bulk copy all the fields from the given point into this object
        \param destPointIndex the destination point index to copy the data from
                srcPointBuffer at srcPointIndex.
        \param srcPointIndex the point index of the PointBuffer to copy from.
        \param srcPointBuffer the source PointBuffer to copy from
        \verbatim embed:rst 
        .. warning::
            
            This is only legal if the source and destination schemas are 
            exactly the same. It is up the caller to ensure this is the case.
            Additionally, if the schemas are the same :cpp:func:`pdal::Schema::getByteSize()` but of 
            different compositions, congratulations :)
        \endverbatim
    */
    inline void copyPointFast(  std::size_t destPointIndex, 
                                std::size_t srcPointIndex, 
                                const PointBuffer& srcPointBuffer)
    {
        const boost::uint8_t* src = srcPointBuffer.getData(srcPointIndex);
        boost::uint8_t* dest = getData(destPointIndex);

        memcpy(dest, src, m_byteSize);

        assert(m_numPoints <= m_capacity);

        return;
    }
    
    /*! bulk copy all the fields from the given point into this object
        \param destPointIndex the destination point index to copy the data from
                srcPointBuffer at srcPointIndex.
        \param srcPointIndex the point index of the PointBuffer to copy from.
        \param srcPointBuffer the source PointBuffer to copy from 
        \param numPoints the number of points to copy
        \verbatim embed:rst 
        .. warning::
            
            This is only legal if the source and destination schemas are 
            exactly the same. It is up the caller to ensure this is the case.
            Additionally, if the schemas are the same :cpp:func:`pdal::Schema::getByteSize()` but of 
            different compositions, congratulations :)
        \endverbatim
    */
    inline void copyPointsFast( std::size_t destPointIndex, 
                                std::size_t srcPointIndex, 
                                const PointBuffer& srcPointBuffer, 
                                std::size_t numPoints)
    {
        const boost::uint8_t* src = srcPointBuffer.getData(srcPointIndex);
        boost::uint8_t* dest = getData(destPointIndex);

        memcpy(dest, src, m_byteSize * numPoints);

        assert(m_numPoints <= m_capacity);

        return;
    }

/** @name Raw Data Access
*/ 
    /// access to the raw byte data at specified pointIndex
    /// @param pointIndex position to start accessing
    inline boost::uint8_t* getData(std::size_t pointIndex) const
    {
        return m_data.get() + m_byteSize * pointIndex;
    }

    /// copies the raw data into your own byte array and sets the size
    /// @param data pointer to your byte array
    /// @param size size of the array in bytes
    void getData(boost::uint8_t** data, std::size_t* size) const;
        
    /// set the data for a single point at given pointIndex from a 
    /// raw byte array
    /// @param data raw byte array
    /// @param pointIndex point position to set data. 
    void setData(boost::uint8_t* data, std::size_t pointIndex);
    
    /// overwrite raw data at a given pointIndex for a given number of byteCount
    /// @param data raw byte array
    /// @param pointIndex position to start writing
    /// @param byteCount number of bytes to overwrite at given position
    void setDataStride(boost::uint8_t* data, std::size_t pointIndex, boost::uint32_t byteCount);

/** @name Metadata
*/ 
    
    /// return  Metadatas const& for the PointBuffer
    inline Metadatas const& getMetadata() const { return m_metadata; }
    
    /// @return Metadatas& for the PointBuffer
    inline Metadatas& getMetadataRef() { return m_metadata; }

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
            offset or the datatype, use the :cpp:func:`pdal::Schema::getPtree()` dumper.
        \endverbatim
    */
    boost::property_tree::ptree toPTree() const;

/** @name private attributes
*/  
private:
    Schema m_schema;
    boost::scoped_array<boost::uint8_t> m_data;
    boost::uint32_t m_numPoints;
    boost::uint32_t m_capacity;    
    Bounds<double> m_bounds;
    
    // We cache m_schema.getByteSize() here because it would end up 
    // being dereferenced for every point read otherwise.
    schema::size_type m_byteSize;
    
    Metadatas m_metadata;
    
    template<class T> T convertDimension(pdal::Dimension const& dim, void* bytes) const;    
    template<typename Target, typename Source> static Target saturation_cast(Source const& src);

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
        throw buffer_error("This dimension has no identified position in a schema.");
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
                    return boost::numeric_cast<T>(*(float const*)p);
                }
                else
                    return boost::numeric_cast<T>(*( double const*)p);
            }
        }

        return *(T const*)( void const*)p;
    }

    return convertDimension<T>(dim, (void *)p);

    
}


PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const PointBuffer&);


} // namespace pdal

#endif
