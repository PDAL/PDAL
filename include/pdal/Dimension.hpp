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

#pragma once

#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/uuid/uuid.hpp>

#include <pdal/pdal_internal.hpp>
#include <pdal/Utils.hpp>

#include <pdal/GlobalEnvironment.hpp>

#include <limits>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(push)
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal
{

namespace dimension
{

/// Dimension flags to denote behaviors of the instance such as 
/// whether to ignore the data or not. Currently, only IsIgnored is 
/// used and respected to any degree by drivers such as 
/// pdal::filters::InPlaceReprojection and pdal::drivers::oci::Writer.
enum Flags
{
    IsIgnored = 0x8,
    ForceKeep = 0x10
};

/// Interpretation for a Dimension denotes what *kind* of data type the 
/// values stored in the dimension should be interpreted as. It can be used
/// in combination with Dimension::getByteSize() to determine the 
/// explicity type/size of the Dimension (such as the ``cstint.h``-style 
/// values `uint32_t` or `int64_t`).
enum Interpretation
{
    SignedInteger,
    UnsignedInteger,
    Float,
    Undefined
};

} // namespace dimension

/*! 
    A Dimension is the description of a single data field in a
    Schema. It is composed of a name, interpretation, a uuid (dimension::id),
    and a size. Upon creation, the dimension::id is set to a random value
    (this can be overridden, and it is expected that each dimension added to
    a Schema have a unique dimension::id. When a dimension is added to a Schema,
    two more properties are also modified: the position (index) of this
    dimension in the schema's list of dimensions, and the byte offset where
    the dimension is stored in the PointBuffer's raw bytes
*/
class PDAL_DLL Dimension
{
public:
    friend class Schema;

    /** @name Constructors
    */
    Dimension() : m_flags(0), m_byteSize(4), m_numericScale(1.0),
        m_numericOffset(0.0), m_interpretation(dimension::SignedInteger)
        {}

    /// Base constructor for Dimension
    /// @param name the name to use for the dimension.
    /// Typically "X" or "Y" or "Interesting Scanner Attribute"
    /// @param interpretation the dimension::Interpretation to use for the
    /// dimension.
    /// @param sizeInBytes the size of the Dimension in bytes. No
    /// less-than-a-byte dimensions are allowed.
    /// @param description a string description of the dimension. (defaults
    /// to empty)
    Dimension(const std::string& name, dimension::Interpretation interpretation,
        size_t sizeInBytes, std::string description = std::string());

    /** @name Equality and comparisons operators
    */
    /// Equality
    bool operator==(const Dimension& other) const
        { return m_uuid == other.m_uuid; }
    /// Inequality
    bool operator!=(const Dimension& other) const
        { return m_uuid != other.m_uuid; }

    bool operator < (Dimension const& dim) const
    {
        return m_offset < dim.m_offset;
    }

    /** @name Attributes
    */
    /// @return the name of this dimension as given at construction time
    std::string const& getName() const
        { return m_name; }

    void setInterpretation(dimension::Interpretation interp)
        { m_interpretation = interp; }

    void setByteSize(size_t byteSize)
        { m_byteSize = byteSize; }

    /// @return the interpretation of this dimension at construction time
    dimension::Interpretation getInterpretation() const
        { return m_interpretation; }

    /// @return dimension attribute flags (isValid, isRead, isWritten,
    /// isIgnored, etc) composition of dimension::Flags
    boost::uint32_t getFlags() const
        { return m_flags; }

    /// sets the dimension attribute flags (isValid, isRead, etc) of
    /// dimension::Flags
    /// @param flags composited dimension::Flags
    void setFlags(uint32_t flags)
        { m_flags = flags; }

    void setIgnored()
        { m_flags |= dimension::IsIgnored; }

    //
    /// @return is this dimension ignored?
    bool isIgnored() const
        { return (m_flags & dimension::IsIgnored); }

    bool forceKeep() const
        { return (m_flags & dimension::ForceKeep); }

    /// @return Number of bytes required to serialize this dimension
    size_t getByteSize() const
       { return m_byteSize; }

    /// @return a string description of the dimension
    std::string getDescription() const
        { return m_description; }

    /// sets the string description for the dimension. Overrides whatever was
    /// given in the constructor.
    /// @param v string to use to set value
    void setDescription(std::string const& v)
        { m_description = v; }

    /// @name Summary and serialization
    /// @return a boost::property_tree::ptree representation
    /// of the Dimension instance
    boost::property_tree::ptree toPTree() const;

    std::string getInterpretationName() const;

    /// @name Identification
    /// @return the dimension::id for the Dimension instance.
    /// This value is the nil UUID by default.
    boost::uuids::uuid const& getUUID() const
        { return m_uuid; }

    /// sets the dimension::id from a string representation of the UUID.
    /// @param id
    void setUUID(std::string const& id)
        { m_uuid = GlobalEnvironment::get().generateUUID(id); }

    /// sets the dimension::id from an existing dimension::id (copied)
    void setUUID(boost::uuids::uuid const& id)
        { m_uuid = id; }

    /// creates and sets the dimension::id for the instance
    void createUUID()
        { m_uuid = GlobalEnvironment::get().generateUUID(); }

    /// @name Namespaces
    /// sets the namespace for this instance
    /// @param name value to set. Typically this is a Stage::getName()
    void setNamespace(std::string const& name)
        { m_namespace = name; }

    /// @return the namespace for this instance
    std::string const& getNamespace() const
        { return m_namespace; }

    /// @return the fully qualified (namespace.name) name for this instance.
    std::string getFQName() const
        { return m_namespace + "." + m_name; }

/** @name Data Scaling
    Scale and offset of Dimension instances are available to describe
    the conversion of these dimensions to floating point values.
*/

    /// @return the numerical scale value for this dimension as a double. The
    /// default value is \b 1.0
    double getNumericScale() const
        { return m_numericScale; }

    /// Sets the numerical scale value for this dimension.
    inline void setNumericScale(double v)
        { m_numericScale = v; }

    /// @return the numerical offset value for this dimension. The default
    /// value is \b 0.0.
    inline double getNumericOffset() const
        { return m_numericOffset; }

    /// Sets the numerical offset value for this dimension.
    inline void setNumericOffset(double v)
        { m_numericOffset = v; }
    
    /// Applies the scale and offset values from the dimension to a the
    /// given value
    /// @param v The value of type T to scale.
    /// @return v scaled by getNumericOffset() and getNumericScale() values.
    /*!    \verbatim embed:rst
        .. note::

            The value ``v`` is casted to a double before math is applied.
        \endverbatim
    */
    double applyScaling(double v) const
       { return v * m_numericScale + m_numericOffset; }

    double removeScaling(double v) const
       { return (v - m_numericOffset) / m_numericScale; }

    /// Removes the scale and offset values from an imprecise double value
    /// @param v The value to descale
    /// @return a value that has been descaled by the Dimension's
    /// getNumericOffset and getNumericScale values
    /*!    \verbatim embed:rst
        .. warning::

            If the value will overflow the give ``T``,
            a std::out_of_range exception will be thrown.
        \endverbatim
    */
    
    /// Return the dimension::Interpretation for a given stdint.h-style
    /// type name such as `int32_t` or `uint8_t`.
    static dimension::Interpretation getInterpretation(
        std::string const& interpretation_name);
    
    // Helper for below.
    template <typename T_OUT, typename T_IN>
    T_OUT convert(void *data)
    {
        T_IN temp = *(T_IN*)(void*)data;
        return Utils::saturation_cast<T_OUT, T_IN>(temp);
    }
    
    /// Converts the a pointer to a value to the data type described 
    /// by the Dimension instance.
    /// @param data a pointer to the data value to be used. The value is 
    /// casted via a Utils::saturation_cast so if it overflows, it will be 
    /// the max (or min) value of the Dimension's type.
    template <class T>
    T convert(void* data) const
    {
        switch (getInterpretation())
        {
        case dimension::SignedInteger:
            switch (getByteSize())
            {
            case 1:
                return convert<T, int8_t>(data);
            case 2:
                return convert<T, int16_t>(data);
            case 4:
                return convert<T, int32_t>(data);
            case 8:
                return convert<T, int64_t>(data);
            default:
                throw buffer_error("convert: Unhandled datatype size "
                    "for SignedInteger");
                break;
            }
        case dimension::UnsignedInteger:
            switch (getByteSize())
            {
            case 1:
                return convert<T, uint8_t>(data);
            case 2:
                return convert<T, uint16_t>(data);
            case 4:
                return convert<T, uint32_t>(data);
            case 8:
                return convert<T, uint64_t>(data);
            default:
                throw buffer_error("convert: Unhandled datatype size "
                    "for UnsignedInteger");
                break;
            }
        case dimension::Float:
            switch (getByteSize())
            {
            case 4:
                return convert<T, float>(data);
            case 8:
                return convert<T, double>(data);
            default:
                throw buffer_error("convert: Unhandled datatype size "
                    "for Float");
                break;
            }
        default:
            throw buffer_error("Undefined interpretation for convert");
        }
        return T(0);
    }

/// @name Private Attributes
private:
    std::string m_name;
    unsigned m_flags;
    size_t m_byteSize;
    std::string m_description;
    double m_min;
    double m_max;
    double m_numericScale;
    double m_numericOffset;
    dimension::Interpretation m_interpretation;
    boost::uuids::uuid m_uuid;
    std::string m_namespace;
    int m_offset;
};
typedef std::shared_ptr<Dimension> DimensionPtr;
typedef std::vector<DimensionPtr> DimensionList;

PDAL_DLL std::ostream& operator<<(std::ostream& os, pdal::Dimension const& d);

} // namespace pdal

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(pop)
#endif

