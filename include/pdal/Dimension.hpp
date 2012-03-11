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

/// A Dimension consists of a name and a datatype.
///
/// When a dimension is added to a Schema, it also gets two more properties: the position (index)
/// of this dimension in the schema's list of dimensions, and the byte offset where the dimension
/// is stored in the PointBuffer's raw bytes
class PDAL_DLL Dimension
{
public:

/// \name Constructors
    Dimension(  std::string const& name, 
                dimension::Interpretation interpretation,
                dimension::size_type sizeInBytes,
                std::string description=std::string(""));
    Dimension(Dimension const& other);

    Dimension& operator=(Dimension const& rhs);

/// \name Equality
    bool operator==(const Dimension& other) const;
    bool operator!=(const Dimension& other) const;

    inline bool operator < (Dimension const& dim) const 
    {
        return m_position < dim.m_position;
    }
    inline bool operator > (Dimension const& dim) const 
    {
        return m_position > dim.m_position;
    }

/// \name Data Access
    std::string const& getName() const;

    // DimensionId::Id getId() const
    // {
    //     return m_id;
    // }
    
    inline dimension::Interpretation getInterpretation() const { return m_interpretation; }

    boost::uint32_t getFlags() const { return m_flags; }
    void setFlags(boost::uint32_t flags) { m_flags = flags; }

    bool isValid() const { return (m_flags != dimension::Invalid); }
    bool isRead() const { return (m_flags & dimension::IsRead) == dimension::IsRead; }
    bool isWritten() const { return (m_flags & dimension::IsWritten) == dimension::IsWritten; }
    bool isIgnored() const { return (m_flags & dimension::IsIgnored) == dimension::IsIgnored; }
    

    /// \return Number of bytes required to serialize this dimension 
    dimension::size_type getByteSize() const
    {
       return m_byteSize;
    }

    inline std::string getDescription() const
    {
        return m_description;
    }
    inline void setDescription(std::string const& v)
    {
        m_description = v;
    }

    /// The minimum value of this dimension as a double
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

    /// The maximum value of this dimension as a double.
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

    /// Gets the numerical scale value for this dimension. If the dimension 
    /// is not finite, the default value is \b 1.0
    /// \return The scaling value for this dimension as a double. It is used in combination 
    /// with the numerical offset value for finite precision dimensions.
    inline double getNumericScale() const
    {
        return m_numericScale;
    }
    /// Sets the numerical scale value for this dimension. If you set a value 
    /// other that \b 0.0, isFinitePrecision for the dimension will also now 
    /// be true.
    inline void setNumericScale(double v)
    {
        m_numericScale = v;
    }

    /// Gets the numerical offset value for this dimension. If the dimension 
    /// is not finite, the default value is \b 0.0
    /// \return The numerical offset value for this dimension.  It is used in combination 
    /// with the numerical scale value for finite precision dimensions
    inline double getNumericOffset() const
    {
        return m_numericOffset;
    }
    /// Sets the numerical offset value for this dimension. If you set a value 
    /// other that \b 0.0, isFinitePrecision for the dimension will also now 
    /// be true.
    inline void setNumericOffset(double v)
    {
        m_numericOffset = v;
    }

    /// Applies the scale and offset values from the dimension to a the given value
    /// \param v The value to scale with the Dimension's NumericOffset and NumericScale values
    /// \return \b v scaled by getNumericOffset() and getNumericScale() values.
    template<class T>
    inline double applyScaling(T const& v) const
    {
        return static_cast<double>(v) * m_numericScale + m_numericOffset;
    }

    /// Removes the scale and offset values from an imprecise double value
    /// \param v The value to descale with the Dimension's NumericScale and NumericOffset values
    /// \return a value that has been descaled by the Dimension's NumericOffset and NumericScale values
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

    /// Gets the endianness of this Dimension (defaults to little)
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

    /// The byte location to start reading/writing
    /// point data from in a composited schema.  liblas::Schema
    /// will set these values for you when liblas::Dimension are
    /// added to the liblas::Schema.
    inline std::size_t getByteOffset() const
    {
        return m_byteOffset;
    }

    inline void setByteOffset(std::size_t v)
    {
        m_byteOffset = v;
    }

    /// The index position of the index.  In a standard ePointFormat0
    /// data record, the X dimension would have a position of 0, while
    /// the Y dimension would have a position of 1, for example.
    inline dimension::size_type getPosition() const
    {
        return m_position;
    }

    inline void setPosition(dimension::size_type v)
    {
        m_position = v;
    }

/// \name Summary and serialization
    /// Outputs a string-based boost::property_tree::ptree representation 
    /// of the Dimension instance
    boost::property_tree::ptree toPTree() const;

    /// Outputs a string representation of the Dimension instance to std::cout
    void dump() const;
    
    std::string getInterpretationName() const;

    dimension::id const& getUUID() const { return m_uuid; }
    void setUUID( std::string const& id);
    void setUUID( dimension::id& id) { m_uuid = id; }
    void createUUID();

    void setNamespace( std::string const& name) { m_namespace = name; }
    std::string const& getNamespace( ) const { return m_namespace; }

    std::string getFQName( ) const { return m_namespace + "." + m_name; }
    
    void setParent( dimension::id const& id) { m_parentDimensionID = id; }
    dimension::id const& getParent( ) const 
    {
        return m_parentDimensionID;
    }
    
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
