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
#include <pdal/Environment.hpp>

#include <boost/property_tree/ptree.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <limits>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(push)
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal
{

namespace dimension {

    
    typedef boost::uuids::uuid id;

    enum Flags
    {
        Invalid   = 0x0,
        IsAdded   = 0x1,
        IsRead    = 0x2,
        IsWritten = 0x4,
        IsIgnored = 0x8
    };

    typedef boost::int32_t size_type;
    
    enum Interpretation
    {
        SignedByte,
        UnsignedByte,
        SignedInteger,
        UnsignedInteger,
        Pointer,
        Float,
        Undefined
    };
    
} // dimension


/// A pdal::Dimension is the description of a single data field in a pdal::Schema.
/// A pdal::Dimension is composed of a name, interpretation, and a size.
/// When a dimension is added to a pdal::Schema, it also gets two more properties: the position (index)
/// of this dimension in the schema's list of dimensions, and the byte offset where the dimension
/// is stored in the PointBuffer's raw bytes
class PDAL_DLL Dimension
{
public:

/** @name Constructors
*/ 
    /// Base constructor for pdal::Dimension
    /// @param name the name to use for the dimension. 
    /// Typically "X" or "Y" or "Interesting Scanner Attribute"
    /// @param interpretation the pdal::dimension::Interpretation to use for the 
    /// dimension.
    /// @param sizeInBytes the size of the pdal::Dimension in bytes. No 
    /// less-than-a-byte dimensions are allowed.
    /// @param description a string description of the dimension.
    Dimension(  std::string const& name, 
                dimension::Interpretation interpretation,
                dimension::size_type sizeInBytes,
                std::string description=std::string(""));
                
    /// Copy constructor
    Dimension(Dimension const& other);
    
    /// Assignment constructor
    Dimension& operator=(Dimension const& rhs);

/** @name Equality and Comparisons
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
    inline std::string const& getName() const { return m_name; }
    
    /// @return the interpretation of this dimension at construction time
    inline dimension::Interpretation getInterpretation() const { return m_interpretation; }
    
    /// @return dimension attribute flags (isValid, isRead, isWritten, isIgnored, etc) 
    /// composition of pdal::dimension::Flags
    boost::uint32_t getFlags() const { return m_flags; }
    
    /// sets the dimension attribute flags (isValid, isRead, etc) of pdal::dimension::Flags
    /// @param flags composited pdal::dimension::Flags 
    void setFlags(boost::uint32_t flags) { m_flags = flags; }
    
    /// @return is the dimension valid?
    bool isValid() const { return (m_flags != dimension::Invalid); }
    
    /// @return should we read this dimension?
    bool isRead() const { return (m_flags & dimension::IsRead) == dimension::IsRead; }
    
    /// @return should we write this dimension?
    bool isWritten() const { return (m_flags & dimension::IsWritten) == dimension::IsWritten; }

    /// @return is this dimension ignored?
    bool isIgnored() const { return (m_flags & dimension::IsIgnored) == dimension::IsIgnored; }
    
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

    /// @return the byte offset of the pdal::Dimension instance within the 
    /// context of a pdal::Schema. pdal::Schema will set this value when 
    /// adding the pdal::Dimension to itself so as to not require calculating 
    /// it for every lookup.
    inline std::size_t getByteOffset() const
    {
        return m_byteOffset;
    }

    /// sets the byte offset of the pdal::Dimension
    /// @param v the value to set
    inline void setByteOffset(std::size_t v)
    {
        m_byteOffset = v;
    }

    /// @return the position of the pdal::Dimension within a pdal::Schema. 
    /// If the instance is not in a pdal::Schema instance, this value is 
    /// initialized to -1. 
    inline dimension::size_type getPosition() const
    {
        return m_position;
    }
    
    /// Sets the position of the pdal::Dimension instance within a pdal::Schema
    inline void setPosition(dimension::size_type v)
    {
        m_position = v;
    }

/// @name Summary and serialization
    /// @return a boost::property_tree::ptree representation 
    /// of the pdal::Dimension instance
    boost::property_tree::ptree toPTree() const;

    /// Outputs a string representation of the Dimension instance to std::cout
    void dump() const;
    
    std::string getInterpretationName() const;

/// @name Identification

    /// @return the pdal::dimension::id for the pdal::Dimension instance. 
    /// This value is the nil UUID by default.
    inline dimension::id const& getUUID() const { return m_uuid; }
    
    /// sets the dimension::id from a string representation of the UUID.
    /// @param id
    void setUUID( std::string const& id);
    
    /// sets the dimension::id from an existing dimension::id (copied)
    inline void setUUID( dimension::id const& id) { m_uuid = id; }
    
    /// creates and sets the dimension::id for the instance
    void createUUID();
    
    /// denotes the parent relationship of this instance to another 
    /// with a given dimension::id
    /// @param id the dimension::id of the parent dimension to this instance
    inline void setParent( dimension::id const& id)
    { 
        m_parentDimensionID = id;
    }
    
    /// @return the dimension::id of the parent dimension to this one.
    inline dimension::id const& getParent( ) const
    {
        return m_parentDimensionID;
    }

/// @name Namespaces
    /// sets the namespace for this instance
    /// @param name value to set. Typically this is a pdal::Stage::getName()
    inline void setNamespace( std::string const& name )
    { 
        m_namespace = name; 
    }
    
    /// @return the namespace for this instance
    inline std::string const& getNamespace( ) const
    { 
        return m_namespace; 
    }

    /// @return the fully qualified (namespace.name) name for this instance.
    inline std::string getFQName( ) const
    { 
        return m_namespace + "." + m_name;
    }
    


/** @name Data Scaling
*/ 
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
            
            If the value will overflow the datatype ``T`` that is given, 
            an std::out_of_range exception will be thrown.
        \endverbatim
    */
    template<class T>
    inline T removeScaling(double const& v) const
    {
        T output = static_cast<T>(Utils::sround((v - m_numericOffset)/ m_numericScale));
        
        if (std::numeric_limits<T>::is_exact) // 
        {
            if (output > (std::numeric_limits<T>::max)())
            {
                std::ostringstream oss;
                oss << "filter.Scaling: scale and/or offset combination causes " 
                       "re-scaled value to be greater than std::numeric_limits::max for dimension '" << getName() << "'. " <<
                       "value is: " << output << " and max() is: " << (std::numeric_limits<T>::max)();        
            } 
            else if (output < (std::numeric_limits<T>::min)() )
            {
                std::ostringstream oss;
                oss << "filter.Scaling: scale and/or offset combination causes " 
                       "re-scaled value to be less than std::numeric_limits::min for dimension '" << getName() << "'. " <<
                       "value is: " << output << " and min() is: " << (std::numeric_limits<T>::min)();
                throw std::out_of_range(oss.str());

            }
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
