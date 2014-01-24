/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS Dimension implementation for C++ libLAS
 * Author:   Howard Butler, hobu.inc@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2010, Howard Butler
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#ifndef PDAL_DIMENSION_HPP_INCLUDED
#define PDAL_DIMENSION_HPP_INCLUDED

#include <pdal/pdal_internal.hpp>
#include <pdal/Utils.hpp>

#include <boost/property_tree/ptree.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/type_traits.hpp>

#include <limits>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(push)
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal
{

namespace dimension
{

/// Explicit, 64-bit UUID for the Dimension. A random one is created 
/// for each new Dimension instance, but it can be explicitly set if 
/// desired.
typedef boost::uuids::uuid id;


/// Dimension flags to denote behaviors of the instance such as 
/// whether to ignore the data or not. Currently, only IsIgnored is 
/// used and respected to any degree by drivers such as 
/// pdal::filters::InPlaceReprojection and pdal::drivers::oci::Writer.
enum Flags
{
    Invalid   = 0x0,
    IsAdded   = 0x1,
    IsRead    = 0x2,
    IsWritten = 0x4,
    IsIgnored = 0x8
};

/// Size type for Dimension. It can be negative, and a ``-1`` value 
/// is used as an indicator of dimension position not being set.
typedef boost::int32_t size_type;


/// Interpretation for a Dimension denotes what *kind* of data type the 
/// values stored in the dimension should be interpreted as. It can be used
/// in combination with Dimension::getByteSize() to determine the 
/// explicity type/size of the Dimension (such as the ``cstint.h``-style 
/// values `uint32_t` or `int64_t`).
enum Interpretation
{
    RawByte,
    SignedInteger,
    UnsignedInteger,
    Pointer,
    Float,
    Undefined
};

} // dimension

/*! 
    
    A Dimension is the description of a single data field in a
    Schema. It is composed of a name, interpretation, a uuid (dimension::id), and a
    size. Upon creation, the dimension::id is set to a random value (this can 
    be overridden, and it is expected that each dimension added to a Schema have a 
    unique dimension::id. When a dimension is added to a Schema, two more
    properties are also modified: the position (index) of this dimension in the schema's list of
    dimensions, and the byte offset where the dimension is stored in the
    PointBuffer's raw bytes

    Some other text goes here that describes something else
*/
class PDAL_DLL Dimension
{
public:

    /** @name Constructors
    */
    /// Base constructor for Dimension
    /// @param name the name to use for the dimension.
    /// Typically "X" or "Y" or "Interesting Scanner Attribute"
    /// @param interpretation the dimension::Interpretation to use for the
    /// dimension.
    /// @param sizeInBytes the size of the Dimension in bytes. No
    /// less-than-a-byte dimensions are allowed.
    /// @param description a string description of the dimension. (defaults to empty)
    Dimension(std::string const& name,
              dimension::Interpretation interpretation,
              dimension::size_type sizeInBytes,
              std::string description=std::string(""));

    /// Copy constructor
    Dimension(Dimension const& other);

    /// Assignment constructor
    Dimension& operator=(Dimension const& rhs);

    /** @name Equality and comparisons operators
    */
    /// Equality
    bool operator==(const Dimension& other) const;
    /// Inequality
    bool operator!=(const Dimension& other) const;

    /// Less than. Determined by getPosition() for sorting.
    inline bool operator < (Dimension const& dim) const
    {
        return m_position < dim.m_position;
    }

    /// Greater than. Determined by getPosition for sorting.
    inline bool operator > (Dimension const& dim) const
    {
        return m_position > dim.m_position;
    }

    /** @name Attributes
    */
    /// @return the name of this dimension as given at construction time
    inline std::string const& getName() const
    {
        return m_name;
    }

    /// @return the interpretation of this dimension at construction time
    inline dimension::Interpretation getInterpretation() const
    {
        return m_interpretation;
    }

    /// @return dimension attribute flags (isValid, isRead, isWritten, isIgnored, etc)
    /// composition of dimension::Flags
    boost::uint32_t getFlags() const
    {
        return m_flags;
    }

    /// sets the dimension attribute flags (isValid, isRead, etc) of dimension::Flags
    /// @param flags composited dimension::Flags
    void setFlags(boost::uint32_t flags)
    {
        m_flags = flags;
    }

    /// @return is the dimension valid?
    bool isValid() const
    {
        return (m_flags != dimension::Invalid);
    }

    /// @return should we read this dimension?
    bool isRead() const
    {
        return (m_flags & dimension::IsRead) == dimension::IsRead;
    }

    /// @return should we write this dimension?
    bool isWritten() const
    {
        return (m_flags & dimension::IsWritten) == dimension::IsWritten;
    }

    /// @return is this dimension ignored?
    bool isIgnored() const
    {
        return (m_flags & dimension::IsIgnored) == dimension::IsIgnored;
    }

    /// @return Number of bytes required to serialize this dimension
    inline dimension::size_type getByteSize() const
    {
        return m_byteSize;
    }

    /// @return a string description of the dimension
    inline std::string getDescription() const
    {
        return m_description;
    }
    /// sets the string description for the dimension. Overrides whatever was
    /// given in the constructor.
    /// @param v string to use to set value
    inline void setDescription(std::string const& v)
    {
        m_description = v;
    }

    /// @return the minimum value of this dimension as a double
    inline double getMinimum() const
    {
        return m_min;
    }

    /*! Sets the minimum value of this dimension as a double.
        \param max The minimum value for this dimension
        \verbatim embed:rst
        .. note::

            The maximum and minimum values are simply data placeholders
            and in most cases will be ``0.0``.
        \endverbatim
    */
    inline void setMinimum(double min)
    {
        m_min = min;
    }

    /// @return the maximum value of this dimension as a double.
    inline double getMaximum() const
    {
        return m_max;
    }

    /*! Sets the maximum value of this dimension as a double.
        \param max The maximum value for this dimension
        \verbatim embed:rst
        .. note::

            The maximum and minimum values are simply data placeholders
            and in most cases will be ``0.0``.
        \endverbatim
    */
    inline void setMaximum(double max)
    {
        m_max = max;
    }

    /// Gets the endianness of this intance (defaults to little)
    inline EndianType getEndianness() const
    {
        return m_endian;
    }

    /// Sets the endianness of this Dimension
    /// \param v EndianType value to set for the dimension
    inline void setEndianness(EndianType v)
    {
        m_endian = v;
    }

    /// @return the byte offset of the Dimension instance within the
    /// context of a Schema. Schema will set this value when
    /// adding the Dimension to itself so as to not require calculating
    /// it for every lookup.
    inline std::size_t getByteOffset() const
    {
        return m_byteOffset;
    }

    /// sets the byte offset of the Dimension
    /// @param v the value to set
    inline void setByteOffset(std::size_t v)
    {
        m_byteOffset = v;
    }

    /// @return the position of the Dimension within a Schema.
    /// If the instance is not in a Schema instance, this value is
    /// initialized to -1.
    inline dimension::size_type getPosition() const
    {
        return m_position;
    }

    /// Sets the position of the Dimension instance within a Schema
    inline void setPosition(dimension::size_type v)
    {
        m_position = v;
    }

/// @name Summary and serialization
    /// @return a boost::property_tree::ptree representation
    /// of the Dimension instance
    boost::property_tree::ptree toPTree() const;

    std::string getInterpretationName() const;

/// @name Identification

    /// @return the dimension::id for the Dimension instance.
    /// This value is the nil UUID by default.
    inline dimension::id const& getUUID() const
    {
        return m_uuid;
    }

    /// sets the dimension::id from a string representation of the UUID.
    /// @param id
    void setUUID(std::string const& id);

    /// sets the dimension::id from an existing dimension::id (copied)
    inline void setUUID(dimension::id const& id)
    {
        m_uuid = id;
    }

    /// creates and sets the dimension::id for the instance
    void createUUID();

    /// denotes the parent relationship of this instance to another
    /// with a given dimension::id. By default, the parent of an instance 
    /// is the nil uuid.
    /// @param id the dimension::id of the parent dimension to this instance
    inline void setParent(dimension::id const& id)
    {
        m_parentDimensionID = id;
    }

    /// @return the dimension::id of the parent dimension to this one.
    inline dimension::id const& getParent() const
    {
        return m_parentDimensionID;
    }

/// @name Namespaces
    /// sets the namespace for this instance
    /// @param name value to set. Typically this is a Stage::getName()
    inline void setNamespace(std::string const& name)
    {
        m_namespace = name;
    }

    /// @return the namespace for this instance
    inline std::string const& getNamespace() const
    {
        return m_namespace;
    }

    /// @return the fully qualified (namespace.name) name for this instance.
    inline std::string getFQName() const
    {
        return m_namespace + "." + m_name;
    }

/** @name Data Scaling
    Scale and offset of Dimension instances are available to describe
    the conversion of these dimensions to floating point values.
*/

    /// @return the numerical scale value for this dimension as a double. The
    /// default value is \b 1.0
    inline double getNumericScale() const
    {
        return m_numericScale;
    }
    /// Sets the numerical scale value for this dimension.
    inline void setNumericScale(double v)
    {
        m_numericScale = v;
    }

    /// @return the numerical offset value for this dimension. The default value is \b 0.0.
    inline double getNumericOffset() const
    {
        return m_numericOffset;
    }
    /// Sets the numerical offset value for this dimension.
    inline void setNumericOffset(double v)
    {
        m_numericOffset = v;
    }
    
    /// Applies the scale and offset values from the dimension to a the given value
    /// @param v The value of type T to scale.
    /// @return v scaled by getNumericOffset() and getNumericScale() values.
    /*!    \verbatim embed:rst
        .. note::

            The value ``v`` is casted to a double before math is applied.
        \endverbatim
    */
    template<class T>
    inline double applyScaling(T const& v) const
    {
        return static_cast<double>(v) * m_numericScale + m_numericOffset;
    }

    /// Removes the scale and offset values from an imprecise double value
    /// @param v The value to descale
    /// @return a value that has been descaled by the Dimension's getNumericOffset and getNumericScale values
    /*!    \verbatim embed:rst
        .. warning::

            If the value will overflow the give ``T``,
            a std::out_of_range exception will be thrown.
        \endverbatim
    */
#if (__GNUC__ == 4 && __GNUC_MINOR__ >= 6 && !defined(_MSC_VER))    
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wsign-compare"
#endif

    template<class T>
    inline T removeScaling(double const& v) const
    {
        double descaled = Utils::sround((v - m_numericOffset)/ m_numericScale);

        T output(0);
        boost::int64_t i64(0);
        boost::uint64_t u64(0);
        if (boost::is_floating_point<T>::value)
        {   
            output = static_cast<T>(descaled);
            return output;
        }
        
        bool bSigned(boost::is_signed<T>::value);
        bool bGreater(false);
        bool bLess(false);
        if (bSigned)
        {
            i64 = static_cast<boost::int64_t>(descaled);
            boost::int64_t mn = static_cast<boost::int64_t>((std::numeric_limits<T>::min)());            
            boost::int64_t mx = static_cast<boost::int64_t>((std::numeric_limits<T>::max)());            
            bGreater = (i64 > mx);
            bLess = (i64 < mn);
            output = static_cast<T>(i64);              
            if (!bGreater && !bLess)
                return output;
        }
        else
        {
            u64 = static_cast<boost::uint64_t>(descaled);
            boost::uint64_t mn = static_cast<boost::uint64_t>((std::numeric_limits<T>::min)());
            boost::uint64_t mx = static_cast<boost::uint64_t>((std::numeric_limits<T>::max)());
            bGreater = (u64 > mx);
            bLess = (u64 < mn);
            output = static_cast<T>(u64);
            if (!bGreater && !bLess)
                return output;
        }

        if (bGreater)
        {
            std::ostringstream oss;
            boost::int64_t out(0);
            if (bSigned)
                out = static_cast<boost::int64_t>(i64);
            else
                out = static_cast<boost::int64_t>(u64);
            
            oss.precision(12);
            oss.setf(std::ios::fixed);
            oss << "Dimension::removeScaling: scale: '" << m_numericScale 
                << "' and/or offset: " << m_numericOffset <<"' combination causes "
                "de-scaled value to be greater than std::numeric_limits::max for dimension '" 
                << getFQName() << "'. " <<
                "(v - offset)/ scale) is: (" 
                << v << " - " << m_numericOffset << ")/" 
                << m_numericScale <<") == '" << out 
                << "' but max() for the datatype is: " 
                << (std::numeric_limits<T>::max)();
            throw std::out_of_range(oss.str());
        }
        else if (bLess)
        {
            std::ostringstream oss;
            boost::int64_t out(0);
            if (bSigned)
                out = static_cast<boost::int64_t>(i64);
            else
                out = static_cast<boost::int64_t>(u64);
            oss.precision(12);
            oss.setf(std::ios::fixed);
            oss << "Dimension::removeScaling: scale: '" << m_numericScale 
                << "' and/or offset: " << m_numericOffset <<"' combination causes "
                "de-scaled value to be less than std::numeric_limits::min for dimension '" 
                << getFQName() << "'. " <<
                "(v - offset)/ scale) is: (" 
                << v << " - " << m_numericOffset << ")/" 
                << m_numericScale <<") == '" << out 
                << "' but min() for the datatype is: " 
                << (std::numeric_limits<T>::min)();

            throw std::out_of_range(oss.str());
        }
        return output;
    }
#if (__GNUC__ == 4 && __GNUC_MINOR__ >= 6 && !defined(_MSC_VER))
# pragma GCC diagnostic pop
#endif
    
    /// Return the dimension::Interpretation for a given stdint.h-style type name 
    /// such as `int32_t` or `uint8_t`.
    static dimension::Interpretation getInterpretation(std::string const& interpretation_name);
    
    
    /// Converts the a pointer to a value to the data type described 
    /// by the Dimension instance.
    /// @param data a pointer to the data value to be used. The value is 
    /// casted via a Utils::saturation_cast so if it overflows, it will be 
    /// the max (or min) value of the Dimension's type.
    template <class T>
    inline T convert(void* data) const

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

        switch (this->getInterpretation())
        {
            case dimension::RawByte:
                u8 = *(boost::uint8_t*)(void*)data;
                output = Utils::saturation_cast<T, boost::uint8_t>(u8);
                break;

            case dimension::SignedInteger:
                if (this->getByteSize() == 1)
                {
                    i8 = *(boost::int8_t*)(void*)data;
                    output = Utils::saturation_cast<T, boost::int8_t>(i8);
                }
                else if (this->getByteSize() == 2)
                {
                    i16 = *(boost::int16_t*)(void*)data;
                    output = Utils::saturation_cast<T, boost::int16_t>(i16);

                }
                else if (this->getByteSize() == 4)
                {
                    i32 = *(boost::int32_t*)(void*)data;
                    output = Utils::saturation_cast<T, boost::int32_t>(i32);
                }
                else if (this->getByteSize() == 8)
                {
                    i64 = *(boost::int64_t*)(void*)data;
                    output = Utils::saturation_cast<T, boost::int64_t>(i64);
                }
                else
                {
                    throw buffer_error("getField::Unhandled datatype size for SignedInteger");
                }
                break;
            case dimension::UnsignedInteger:
                if (this->getByteSize() == 1)
                {
                    u8 = *(boost::uint8_t*)(void*)data;
                    output = Utils::saturation_cast<T, boost::uint8_t>(u8);
                }
                else if (this->getByteSize() == 2)
                {
                    u16 = *(boost::uint16_t*)(void*)data;
                    output = Utils::saturation_cast<T, boost::uint16_t>(u16);

                }
                else if (this->getByteSize() == 4)
                {
                    u32 = *(boost::uint32_t*)(void*)data;
                    output = Utils::saturation_cast<T, boost::uint32_t>(u32);
                }
                else if (this->getByteSize() == 8)
                {
                    u64 = *(boost::uint64_t*)(void*)data;
                    output = Utils::saturation_cast<T, boost::uint64_t>(u64);
                }
                else
                {
                    throw buffer_error("getField::Unhandled datatype size for UnsignedInteger");
                }

                break;
            case dimension::Float:
                if (this->getByteSize() == 4)
                {
                    flt = *(float*)(void*)data;
                    output = Utils::saturation_cast<T, float>(flt);
                }
                else if (this->getByteSize() == 8)
                {
                    dbl = *(double*)(void*)data;
                    output = Utils::saturation_cast<T, double>(dbl);
                }
                else
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

/// @name Private Attributes
private:
    std::string m_name;
    boost::uint32_t m_flags;
    EndianType m_endian;
    dimension::size_type m_byteSize;
    std::string m_description;
    double m_min;
    double m_max;
    double m_numericScale;
    double m_numericOffset;
    dimension::size_type m_byteOffset;
    dimension::size_type m_position;
    dimension::Interpretation m_interpretation;
    dimension::id m_uuid;
    std::string m_namespace;
    dimension::id m_parentDimensionID;
};


PDAL_DLL std::ostream& operator<<(std::ostream& os, pdal::Dimension const& d);

} // namespace pdal


#ifdef PDAL_COMPILER_MSVC
#  pragma warning(pop)
#endif


#endif // PDAL_DIMENSION_HPP_INCLUDED
