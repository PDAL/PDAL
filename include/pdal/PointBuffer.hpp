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


#include <boost/cstdint.hpp>
#include <boost/scoped_array.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include <pdal/pdal_internal.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/Schema.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/third/nanoflann.hpp>

#include <vector>

namespace pdal
{
    namespace pointbuffer
    {
        typedef boost::uuids::uuid id;
        typedef std::vector<boost::uint8_t>::size_type PointBufferByteSize;

        typedef boost::interprocess::allocator<boost::uint8_t, boost::interprocess::managed_shared_memory::segment_manager>     ShmemAllocator; 
        typedef boost::container::vector<boost::uint8_t, ShmemAllocator> PointBufferVector;

        enum Orientation
        {
            POINT_INTERLEAVED = 1,
            DIMENSION_INTERLEAVED = 2,
            UNKNOWN_INTERLEAVED = 256
        };    

    } // pointbuffer

    
/// A PointBuffer is the object that is passed through pdal::Stage instances
/// to form a pipeline. A PointBuffer is composed of a pdal::Schema that determines
/// the layout of the data contained within, along with a dictionary of pdal::Metadata
/// entries. The capacity of a PointBuffer is determined by the number of points
/// it contains, and the number of points possible in a PointBuffer is limited to
/// std::numeric_limits<boost::uint32_t>::max().  Underneath the covers, a PointBuffer
/// is simply the composed array of bytes described in the pdal::Schema. You can
/// operate on the raw bytes if you need to, but PointBuffer provides a number of
/// convienence methods to make things easier. 
/*! 
    \verbatim embed:rst
    .. note::

        The arrangement of PointBuffer's bytes might either be point-interleaved or
        dimension-interleaved, with point-interleave being the default organization.
        If you are directly modifying a PointBuffer's bytes, you must respect the 
        :cpp:class:`pdal::pointbuffer::Orientation`. 
        
    \endverbatim
*/    
class PDAL_DLL PointBuffer
{
public:

    /** @name Constructors
    */
    /*! Base constructor for pdal::PointBuffer.
        \param schema pdal::Schema instance to use to describe the layout. It is copied.
        \param capacity size of the pdal::PointBuffer in number of points.
    */
    PointBuffer(const Schema& schema, boost::uint32_t capacity=65536);

    /// Copy constructor. The data array is simply memcpy'd.
    PointBuffer(const PointBuffer&);

    /// Assignment constructor.
    PointBuffer& operator=(const PointBuffer&);

    /// Destructor.
    ~PointBuffer();
    
    /** @name Attribute access
    */
    /*! @return the pdal::Bounds instance associated with this pdal::PointBuffer.
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

    /// @return the number of points that are set for the PointBuffer. It is
    /// an arbitrary number that must be <= getCapacity() is the number of
    /// active points for the PointBuffer
    inline boost::uint32_t getNumPoints() const
    {
        return m_numPoints;
    }

    /// sets the number of active points for the PointBuffer.
    /// @param v number of points to set.
    inline void setNumPoints(boost::uint32_t v)
    {
        assert(v <= m_capacity);
        m_numPoints = v;
    }

    /*! @return the maximum number of points the PointBuffer can hold.
        \verbatim embed:rst
        .. note::

            This value is given at construction time, and it in conjunction
            with the given :cpp:class:`pdal::Schema` determine the size of the raw byte
            buffer that contains the point data. It can be changed by 
            a call to :cpp:func:`pdal::PointBuffer::resize`.
        \endverbatim
    */
    inline boost::uint32_t const& getCapacity() const
    {
        return m_capacity;
    }

    /// A const reference to the internally copied pdal::Schema instance that
    /// was given at construction time.
    const Schema& getSchema() const
    {
        return m_schema;
    }

    /// @return the size of the currently allocated raw byte array
    inline pointbuffer::PointBufferByteSize getBufferByteLength() const
    {
        return m_data.size();
    }

    /// @return the size of the theoretically filled raw byte array.
    /// Should be equivalent to PointBuffer::getCapacity() * PointBuffer::getSchema()::getByteSize().
    inline pointbuffer::PointBufferByteSize getBufferByteCapacity() const
    {
        return static_cast<pointbuffer::PointBufferByteSize>(m_byteSize) * static_cast<pointbuffer::PointBufferByteSize>(m_capacity);
    }

    /** @name Point data access
    */
    /*! fetch the value T for a given :cpp:class:`pdal::Dimension` dim at pointIndex `i`.
        \param dim  a pdal::Dimension instance describing the dimension to select
        \param pointIndex the point index of the PointBuffer to select.
        \verbatim embed:rst
        .. warning::

            If the data type of T is not the same as described in :cpp:class:`pdal::Dimension`,
            the data value will be casted into the appropriate type. In some
            situations this may not be what you want. In situations where the T
            is smaller than the datatype given by `dim`, the return value T
            will simply be saturated.
        \endverbatim
    */
    template<class T> T const& getField(Dimension const& dim, boost::uint32_t pointIndex) const;

    /*! set the value T for a given  :cpp:class:`pdal::Dimension` dim at pointIndex i.
        \param dim a pdal::Dimension instance describing the dimension to select
        \param pointIndex the point index of the PointBuffer to select.
        \param value the T value to set
        \verbatim embed:rst
        .. warning::

            If the data type of T is not the same as described in  :cpp:class:`pdal::Dimension`,
            the data value will be casted into the appropriate type. In some
            situations this may not be what you want. In situations where the T
            is smaller than the datatype given by `dim`, the return value T
            will simply be saturated.
        \endverbatim
    */
    template<class T> void setField(Dimension const& dim, boost::uint32_t pointIndex, T value);

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
    inline void copyPointFast(boost::uint32_t destPointIndex,
                              boost::uint32_t srcPointIndex,
                              const PointBuffer& srcPointBuffer)
    {
        const boost::uint8_t* src = srcPointBuffer.getData(srcPointIndex);
        boost::uint8_t* dest = getData(destPointIndex);
        
        assert(srcPointBuffer.getSchema().getByteSize() == m_byteSize);
        
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
    inline void copyPointsFast(boost::uint32_t destPointIndex,
                               boost::uint32_t srcPointIndex,
                               const PointBuffer& srcPointBuffer,
                               boost::uint32_t numPoints)
    {
        const boost::uint8_t* src = srcPointBuffer.getData(srcPointIndex);
        boost::uint8_t* dest = getData(destPointIndex);

        assert(m_numPoints <= m_capacity);
        memcpy(dest, src, m_byteSize * numPoints);
        return;
    }

    /** @name Raw Data Access
    */
    /// access to the raw byte data at specified pointIndex
    /// @param pointIndex position to start accessing
    inline boost::uint8_t* getData(boost::uint32_t pointIndex) const
    {
        return const_cast<boost::uint8_t*>(&(m_data.front())) + m_byteSize * pointIndex;
    }

    /// copies the raw data into your own byte array and sets the size
    /// @param data pointer to your byte array
    /// @param size size of the array in bytes
    void getData(boost::uint8_t** data, boost::uint64_t* size) const;

    /// set the data for a single point at given pointIndex from a
    /// raw byte array
    /// @param data raw byte array
    /// @param pointIndex point position to set data.
    void setData(boost::uint8_t* data, boost::uint32_t pointIndex);

    /// overwrite raw data at a given pointIndex for a given number of byteCount
    /// @param data raw byte array
    /// @param pointIndex position to start writing
    /// @param byteCount number of bytes to overwrite at given position
    void setDataStride(boost::uint8_t* data, boost::uint32_t pointIndex, boost::uint32_t byteCount);

    /** @name Metadata
    */
    /// return  Metadata const& for the PointBuffer
    inline Metadata const& getMetadata() const
    {
        return m_metadata;
    }

    /// @return Metadata& for the PointBuffer
    inline Metadata& getMetadataRef()
    {
        return m_metadata;
    }

    /** @name Memory Operations
    */
    /// Reallocates a new data buffer with the given schema
    /// @param new_schema The new schema to use.
    void reset(Schema const& new_schema);
    
    /// Resizes the PointBuffer to the given capacity. If the 
    /// PointBuffer is already big enough to hold all of the bytes, 
    /// it is not reallocated unless bExact is true
    /// @param capacity The new number of points to use for the instance
    /// @param bExact Whether or not to exactly resize the PointBuffer instance 
    /// with the given point size and force a new reallocation of the data buffer. 
    void resize(boost::uint32_t const& capacity, bool bExact=false);
    
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

    /// Copies dimensions from the given PointBuffer that have both 
    /// similar names and data types. Create a schema::DimensionMap 
    /// and adjust as necessary before utilizing this method to copy 
    /// dimensions from one PointBuffer instance to another.
    static void copyLikeDimensions( PointBuffer const& source, 
                                    PointBuffer& destination, 
                                    schema::DimensionMap const& dimensions,
                                    boost::uint32_t source_starting_position, 
                                    boost::uint32_t destination_starting_position,
                                    boost::uint32_t howMany);
    
    static void scaleData(PointBuffer const& source_buffer,
                          PointBuffer& destination_buffer,
                          Dimension const& source_dimension,
                          Dimension const& destination_dimension,
                          boost::uint32_t source_index,
                          boost::uint32_t destination_index);

    double applyScaling(Dimension const& d,
                        std::size_t pointIndex) const;
                            
    /** @name private attributes
    */
protected:
    Schema m_schema;
    std::vector<boost::uint8_t> m_data;
    boost::uint32_t m_numPoints;
    boost::uint32_t m_capacity;
    Bounds<double> m_bounds;

    // We cache m_schema.getByteSize() here because it would end up
    // being dereferenced for every point read otherwise.
    schema::size_type m_byteSize;

    Metadata m_metadata;
    boost::interprocess::managed_shared_memory *m_segment;
    pointbuffer::id m_uuid;
    

    template<class T> static void scale(Dimension const& source_dimension,
                                 Dimension const& destination_dimension,
                                 T& value);
};

template <class T>
inline void PointBuffer::setField(pdal::Dimension const& dim, boost::uint32_t pointIndex, T value)
{
    if (dim.getPosition() == -1)
    {
        // this is a little harsh, but we'll keep it for now as we shake things out
        throw buffer_error("This dimension has no identified position in a schema. Use the setRawField method to access an arbitrary byte position.");
    }
    
    pointbuffer::PointBufferByteSize point_start_byte_position = static_cast<pointbuffer::PointBufferByteSize>(pointIndex) * static_cast<pointbuffer::PointBufferByteSize>(m_byteSize); 
    pointbuffer::PointBufferByteSize offset = point_start_byte_position + static_cast<pointbuffer::PointBufferByteSize>(dim.getByteOffset());

#ifdef DEBUG
    assert(offset + sizeof(T) <= getBufferByteSize());
#endif

    boost::uint8_t* p = (boost::uint8_t*)&(m_data.front()) + offset;

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
inline  T const& PointBuffer::getField(pdal::Dimension const& dim, boost::uint32_t pointIndex) const
{
    if (dim.getPosition() == -1)
    {
        // this is a little harsh, but we'll keep it for now as we shake things out
        throw buffer_error("This dimension has no identified position in a schema.");
    }

    pointbuffer::PointBufferByteSize point_start_byte_position = static_cast<pointbuffer::PointBufferByteSize>(pointIndex) * static_cast<pointbuffer::PointBufferByteSize>(m_byteSize); 
    boost::uint64_t offset = point_start_byte_position + static_cast<pointbuffer::PointBufferByteSize>(dim.getByteOffset());

#ifdef DEBUG
    // This test ends up being somewhat expensive when run for every field 
    // for every point. 
    
    if (offset + sizeof(T) > getBufferByteSize())
    {
        std::ostringstream oss;
        oss << "Offset for given dimension is off the end of the buffer!";
        throw buffer_error(oss.str());
    }

    assert(offset + sizeof(T) <= getBufferByteSize() );
#endif

    boost::uint8_t const* p = (boost::uint8_t const*)&(m_data.front()) + offset;
    T const& output = *(T const*)(void const*)p;
    return output;
}


#ifdef PDAL_COMPILER_MSVC
// the writeScaledData function causes lots of conversion warnings which
// are real but which we will ignore: blame the filter's user for casting badly
#  pragma warning(push)
#  pragma warning(disable: 4244)  // conversion from T1 to T2, possible loss of data
#endif

inline void PointBuffer::scaleData(PointBuffer const& source,
                            PointBuffer& destination,
                            Dimension const& source_dimension,
                            Dimension const& destination_dimension,
                            boost::uint32_t source_index,
                            boost::uint32_t destination_index)
{
    if (source_dimension.getInterpretation() == dimension::Float)
    {
        if (source_dimension.getByteSize() == 4)
        {
            float v = source.getField<float>(source_dimension, source_index);
            scale(source_dimension,
                  destination_dimension,
                  v);
            if (destination_dimension.getInterpretation() == dimension::SignedInteger)
            {
                if (destination_dimension.getByteSize() == 1)
                    destination.setField<boost::int8_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 2)
                    destination.setField<boost::int16_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 4)
                    destination.setField<boost::int32_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<boost::int64_t>(destination_dimension, destination_index, v);
                else
                    throw pdal_error("Unable to convert dimension::Float to dimension::SignedInteger of size >8");
                return;
            }
        }
        else if (source_dimension.getByteSize() == 8)
        {
            double v = source.getField<double>(source_dimension, source_index);
            scale(source_dimension,
                  destination_dimension,
                  v);
            if (destination_dimension.getInterpretation() == dimension::SignedInteger)
            {
                if (destination_dimension.getByteSize() == 1)
                    destination.setField<boost::int8_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 2)
                    destination.setField<boost::int16_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 4)
                    destination.setField<boost::int32_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<boost::int64_t>(destination_dimension, destination_index, v);
                else
                    throw pdal_error("Unable to convert dimension::Float to dimension::SignedInteger of size >8");
                return;
            }
        }
        else
        {
            std::ostringstream oss;
            oss << "Unable to interpret Float of size '" << source_dimension.getByteSize() <<"'";
            throw pdal_error(oss.str());
        }
    }

    if (source_dimension.getInterpretation() == dimension::SignedInteger)
    {

        if (destination_dimension.getInterpretation() == dimension::SignedInteger)
        {
            if (source_dimension.getByteSize() == 1)
            {
                boost::int8_t v = source.getField<boost::int8_t>(source_dimension, source_index);
                scale(source_dimension,
                      destination_dimension,
                      v);
                if (destination_dimension.getByteSize() == 1)
                    destination.setField<boost::int8_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 2)
                    destination.setField<boost::int16_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 4)
                    destination.setField<boost::int32_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<boost::int64_t>(destination_dimension, destination_index, v);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 1 to SignedInteger of size > 8");
                return;
            }
            else if (source_dimension.getByteSize() == 2)
            {
                boost::int16_t v = source.getField<boost::int16_t>(source_dimension, source_index);
                scale(source_dimension,
                      destination_dimension,
                      v);
                if (destination_dimension.getByteSize() == 1)
                    destination.setField<boost::int8_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 2)
                    destination.setField<boost::int16_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 4)
                    destination.setField<boost::int32_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<boost::int64_t>(destination_dimension, destination_index, v);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 2 to SignedInteger of size > 8");
                return;
            }
            else if (source_dimension.getByteSize() == 4)
            {
                boost::int32_t v = source.getField<boost::int32_t>(source_dimension, source_index);
                scale(source_dimension,
                      destination_dimension,
                      v);
                if (destination_dimension.getByteSize() == 1)
                    destination.setField<boost::int8_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 2)
                    destination.setField<boost::int16_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 4)
                    destination.setField<boost::int32_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<boost::int64_t>(destination_dimension, destination_index, v);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 4 to SignedInteger of size > 8");
                return;
            }
            else if (source_dimension.getByteSize() == 8)
            {
                boost::int64_t v = source.getField<boost::int64_t>(source_dimension, source_index);
                scale(source_dimension,
                      destination_dimension,
                      v);
                if (destination_dimension.getByteSize() == 1)
                    destination.setField<boost::int8_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 2)
                    destination.setField<boost::int16_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 4)
                    destination.setField<boost::int32_t>(destination_dimension, destination_index, v);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<boost::int64_t>(destination_dimension, destination_index, v);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 8 to SignedInteger of size > 8");
                return;
            }
            else
            {
                throw pdal_error("Unable to convert dimension::SignedInteger >8 to dimension::SignedInteger of size >8");
            }

        }
        else if (destination_dimension.getInterpretation() == dimension::Float)
        {

            if (source_dimension.getByteSize() == 1)
            {
                boost::int8_t v = source.getField<boost::int8_t>(source_dimension, source_index);
                double d = source_dimension.applyScaling<double>(v);
                d = destination_dimension.removeScaling<double>(d);
                if (destination_dimension.getByteSize() == 4)
                    destination.setField<float>(destination_dimension, destination_index, (float)d);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<double>(destination_dimension, destination_index, d);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 1 to Float of size > 8");
                return;
            }
            else if (source_dimension.getByteSize() == 2)
            {
                boost::int16_t v = source.getField<boost::int16_t>(source_dimension, source_index);
                double d = source_dimension.applyScaling<double>(v);
                d = destination_dimension.removeScaling<double>(d);
                if (destination_dimension.getByteSize() == 4)
                    destination.setField<float>(destination_dimension, destination_index, (float)d);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<double>(destination_dimension, destination_index, d);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 2 to Float of size > 8");
                return;
            }
            else if (source_dimension.getByteSize() == 4)
            {
                boost::int32_t v = source.getField<boost::int32_t>(source_dimension, source_index);
                double d = source_dimension.applyScaling<double>(v);
                d = destination_dimension.removeScaling<double>(d);
                if (destination_dimension.getByteSize() == 4)
                    destination.setField<float>(destination_dimension, destination_index, (float)d);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<double>(destination_dimension, destination_index, d);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 4 to Float of size > 8");
                return;
            }
            else if (source_dimension.getByteSize() == 8)
            {
                boost::int64_t v = source.getField<boost::int64_t>(source_dimension, source_index);
                double d = source_dimension.applyScaling<double>(v);
                d = destination_dimension.removeScaling<double>(d);
                if (destination_dimension.getByteSize() == 4)
                    destination.setField<float>(destination_dimension, destination_index, (float)d);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<double>(destination_dimension, destination_index, d);
                else
                    throw pdal_error("Unable to convert dimension from SignedInteger 8 to Float of size > 8");
                return;
            }
            else
            {
                throw pdal_error("Unable to convert dimension::SignedInteger >8 to dimension::Float of size >8");
            }

        }
    }

    if (source_dimension.getInterpretation() == dimension::UnsignedInteger)
    {

        if (destination_dimension.getInterpretation() == dimension::UnsignedInteger)
        {
            if (source_dimension.getByteSize() == 1)
            {
                boost::uint8_t v = source.getField<boost::uint8_t>(source_dimension, source_index);
                double d = source_dimension.applyScaling<double>(v);

                if (destination_dimension.getByteSize() == 1)
                {
                    boost::uint8_t i = destination_dimension.removeScaling<boost::uint8_t>(d);
                    destination.setField<boost::uint8_t>(destination_dimension, destination_index, i);
                }
                else if (destination_dimension.getByteSize() == 2)
                {
                    boost::uint16_t i = destination_dimension.removeScaling<boost::uint16_t>(d);
                    destination.setField<boost::uint16_t>(destination_dimension, destination_index, i);
                }
                else if (destination_dimension.getByteSize() == 4)
                {
                    boost::uint32_t i = destination_dimension.removeScaling<boost::uint32_t>(d);
                    destination.setField<boost::uint32_t>(destination_dimension, destination_index, i);
                }
                else if (destination_dimension.getByteSize() == 8)
                {
                    boost::uint64_t i = destination_dimension.removeScaling<boost::uint64_t>(d);
                    destination.setField<boost::uint64_t>(destination_dimension, destination_index, i);
                }
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 8 to UnsignedInteger of size > 8");
                return;
            }
            else if (source_dimension.getByteSize() == 2)
            {
                boost::uint16_t v = source.getField<boost::uint16_t>(source_dimension, source_index);
                double d = source_dimension.applyScaling<double>(v);

                if (destination_dimension.getByteSize() == 1)
                {
                    boost::uint8_t i = destination_dimension.removeScaling<boost::uint8_t>(d);
                    destination.setField<boost::uint8_t>(destination_dimension, destination_index, i);
                }
                else if (destination_dimension.getByteSize() == 2)
                {
                    boost::uint16_t i = destination_dimension.removeScaling<boost::uint16_t>(d);
                    destination.setField<boost::uint16_t>(destination_dimension, destination_index, i);
                }
                else if (destination_dimension.getByteSize() == 4)
                {
                    boost::uint32_t i = destination_dimension.removeScaling<boost::uint32_t>(d);
                    destination.setField<boost::uint32_t>(destination_dimension, destination_index, i);
                }
                else if (destination_dimension.getByteSize() == 8)
                {
                    boost::uint64_t i = destination_dimension.removeScaling<boost::uint64_t>(d);
                    destination.setField<boost::uint64_t>(destination_dimension, destination_index, i);
                }
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 2 to UnsignedInteger of size > 8");
                return;
            }
            else if (source_dimension.getByteSize() == 4)
            {
                boost::uint32_t v = source.getField<boost::uint32_t>(source_dimension, source_index);
                double d = source_dimension.applyScaling<double>(v);

                if (destination_dimension.getByteSize() == 1)
                {
                    boost::uint8_t i = destination_dimension.removeScaling<boost::uint8_t>(d);
                    destination.setField<boost::uint8_t>(destination_dimension, destination_index, i);
                }
                else if (destination_dimension.getByteSize() == 2)
                {
                    boost::uint16_t i = destination_dimension.removeScaling<boost::uint16_t>(d);
                    destination.setField<boost::uint16_t>(destination_dimension, destination_index, i);
                }
                else if (destination_dimension.getByteSize() == 4)
                {
                    boost::uint32_t i = destination_dimension.removeScaling<boost::uint32_t>(d);
                    destination.setField<boost::uint32_t>(destination_dimension, destination_index, i);
                }
                else if (destination_dimension.getByteSize() == 8)
                {
                    boost::uint64_t i = destination_dimension.removeScaling<boost::uint64_t>(d);
                    destination.setField<boost::uint64_t>(destination_dimension, destination_index, i);
                }
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 4 to UnsignedInteger of size > 8");
                return;
            }
            else if (source_dimension.getByteSize() == 8)
            {
                boost::uint64_t v = source.getField<boost::uint64_t>(source_dimension, source_index);
                double d = source_dimension.applyScaling<double>(v);

                if (destination_dimension.getByteSize() == 1)
                {
                    boost::uint8_t i = destination_dimension.removeScaling<boost::uint8_t>(d);
                    destination.setField<boost::uint8_t>(destination_dimension, destination_index, i);
                }
                else if (destination_dimension.getByteSize() == 2)
                {
                    boost::uint16_t i = destination_dimension.removeScaling<boost::uint16_t>(d);
                    destination.setField<boost::uint16_t>(destination_dimension, destination_index, i);
                }
                else if (destination_dimension.getByteSize() == 4)
                {
                    boost::uint32_t i = destination_dimension.removeScaling<boost::uint32_t>(d);
                    destination.setField<boost::uint32_t>(destination_dimension, destination_index, i);
                }
                else if (destination_dimension.getByteSize() == 8)
                {
                    boost::uint64_t i = destination_dimension.removeScaling<boost::uint64_t>(d);
                    destination.setField<boost::uint64_t>(destination_dimension, destination_index, i);
                }
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 8 to UnsignedInteger of size > 8");
                return;
            }
            else
            {
                throw pdal_error("Unable to convert dimension::UnsignedInteger >8 to dimension::UnsignedInteger of size >8");
            }

        }
        else if (destination_dimension.getInterpretation() == dimension::Float)
        {

            if (source_dimension.getByteSize() == 1)
            {
                boost::uint8_t v = source.getField<boost::uint8_t>(source_dimension, source_index);
                double d = source_dimension.applyScaling<double>(v);
                d = destination_dimension.removeScaling<double>(d);
                if (destination_dimension.getByteSize() == 4)
                    destination.setField<float>(destination_dimension, destination_index, (float)d);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<double>(destination_dimension, destination_index, d);
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 1 to Float of size > 8");
                return;
            }
            else if (source_dimension.getByteSize() == 2)
            {
                boost::uint16_t v = source.getField<boost::uint16_t>(source_dimension, source_index);
                double d = source_dimension.applyScaling<double>(v);
                d = destination_dimension.removeScaling<double>(d);
                if (destination_dimension.getByteSize() == 4)
                    destination.setField<float>(destination_dimension, destination_index, (float)d);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<double>(destination_dimension, destination_index, d);
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 2 to Float of size > 8");
                return;
            }
            else if (source_dimension.getByteSize() == 4)
            {
                boost::uint32_t v = source.getField<boost::uint32_t>(source_dimension, source_index);
                double d = source_dimension.applyScaling<double>(v);
                d = destination_dimension.removeScaling<double>(d);
                if (destination_dimension.getByteSize() == 4)
                    destination.setField<float>(destination_dimension, destination_index, (float)d);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<double>(destination_dimension, destination_index, d);
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 4 to Float of size > 8");
                return;
            }
            else if (source_dimension.getByteSize() == 8)
            {
                boost::uint64_t v = source.getField<boost::uint64_t>(source_dimension, source_index);
                double d = source_dimension.applyScaling<double>(v);
                d = destination_dimension.removeScaling<double>(d);
                if (destination_dimension.getByteSize() == 4)
                    destination.setField<float>(destination_dimension, destination_index, (float)d);
                else if (destination_dimension.getByteSize() == 8)
                    destination.setField<double>(destination_dimension, destination_index, d);
                else
                    throw pdal_error("Unable to convert dimension from UnsignedInteger 8 to Float of size > 8");
                return;
            }
            else
            {
                throw pdal_error("Unable to convert dimension::UnsignedInteger >8 to dimension::Float of size >8");
            }

        }
    }

    throw pdal_error("Dimension data type unable to be scaled because it is Undefined");
}

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(pop)
#endif


#ifdef PDAL_COMPILER_MSVC
// when template T is know, std::numeric_limits<T>::is_exact is a constant...
#  pragma warning(push)
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif
// #pragma GCC diagnostic ignored "-Wfloat-equal"
// #pragma GCC diagnostic push

template <class T>
inline void PointBuffer::scale(Dimension const& source_dimension,
                           Dimension const& destination_dimension,
                           T& value)
{

    double v = static_cast<double>(value);
    double out = (v*source_dimension.getNumericScale() + source_dimension.getNumericOffset() - destination_dimension.getNumericOffset())/destination_dimension.getNumericScale();

    T output = static_cast<T>(out);

    if (std::numeric_limits<T>::is_exact) //
    {
        if (Utils::compare_distance<T>(output, (std::numeric_limits<T>::max)()))
        {
            std::ostringstream oss;
            oss << "PointBuffer::scale: scale and/or offset combination causes "
                "re-scaled value to be greater than std::numeric_limits::max for dimension '" << destination_dimension.getName() << "'. " <<
                "value is: " << output << " and max() is: " << (std::numeric_limits<T>::max)();
        }
        else if (Utils::compare_distance<T>(output, (std::numeric_limits<T>::min)()))
        {
            std::ostringstream oss;
            oss << "PointBuffer::scale: scale and/or offset combination causes "
                "re-scaled value to be less than std::numeric_limits::min for dimension '" << destination_dimension.getName() << "'. " <<
                "value is: " << output << " and min() is: " << (std::numeric_limits<T>::min)();
            throw std::out_of_range(oss.str());

        }
    }
    value = output;

    return;
}


class IndexedPointBuffer : public PointBuffer
{
public:
    IndexedPointBuffer(const Schema& schema, boost::uint32_t capacity=65536);
    IndexedPointBuffer(IndexedPointBuffer const& buffer); 
    IndexedPointBuffer(PointBuffer const& buffer); 
    ~IndexedPointBuffer();
    
    // Satisfy nanoflann::DatasetAdapter
    inline size_t kdtree_get_point_count() const 
    { 
        return PointBuffer::getNumPoints();
    }

    inline double kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim == 0)
            return applyScaling(*m_dimX, idx);
        if (dim == 1)
            return applyScaling(*m_dimY, idx);
        if (dim == 2)
            return applyScaling(*m_dimZ, idx);
        
        std::stringstream oss;
        oss << "dimension '" << dim << "' is out of range 0-2";
        throw std::runtime_error(oss.str());
    }

    double kdtree_distance(const double *p1, const size_t idx_p2,size_t size) const
    {
        const double d0 = applyScaling(*m_dimX, idx_p2) - applyScaling(*m_dimX, size-1);
        const double d1 = applyScaling(*m_dimY, idx_p2) - applyScaling(*m_dimY, size-1);

    // const T d0=p1[0]-pts[idx_p2].x;
    // const T d1=p1[1]-pts[idx_p2].y;
    // const T d2=p1[2]-pts[idx_p2].z;
        double output(d0*d0+d1*d1);
        if (m_is3D)
        {
            const double d2 = applyScaling(*m_dimZ, idx_p2) - applyScaling(*m_dimZ, size-1);
            output += d2*d2;
        }
    return output;
    }
    
    template <class BBOX> bool kdtree_get_bbox(BBOX &bb) const ;

 
    std::vector<size_t> neighbors(double const& x, double const& y, double const& z, double distance, boost::uint32_t count=1);
    std::vector<size_t> radius(double const& x, double const& y, double const& z, double const& r);
    
    // /// Copy constructor. The data array is simply memcpy'd.
    // IndexedPointBuffer(const IndexedPointBuffer&) : PointBuffer(buffer) {}
    // 
    // /// Assignment constructor.
    // PointBuffer& operator=(const PointBuffer&) { return IndexedPointBuffer};
    void build(bool b3D=true);

private:
    pdal::Dimension const* m_dimX;
    pdal::Dimension const* m_dimY;
    pdal::Dimension const* m_dimZ;
    bool m_is3D;
    void setCoordinateDimensions();

    typedef nanoflann::KDTreeSingleIndexAdaptor<
                    nanoflann::L2_Adaptor<double, IndexedPointBuffer> ,
                    IndexedPointBuffer,
                    -1 /* dim */
                    > my_kd_tree_t;

    my_kd_tree_t* m_index;
};

template <class BBOX> bool IndexedPointBuffer::kdtree_get_bbox(BBOX &bb) const 
{
    pdal::Bounds<double> const& bounds = getSpatialBounds();
    if (bounds.empty())
        return false;
    
    size_t nDims = m_is3D && m_dimZ ? 3 : 2;
    for (size_t i=0; i < nDims; ++i)
    {
        bb[i].low = bounds.getMinimum(i);
        bb[i].high = bounds.getMaximum(i);
    }
    return true;
    
}   
#ifdef PDAL_COMPILER_MSVC
#  pragma warning(pop)
#endif
// #pragma GCC diagnostic pop

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const PointBuffer&);


} // namespace pdal

#endif
