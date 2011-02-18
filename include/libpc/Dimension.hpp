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

#ifndef LIBLAS_DIMENSION_HPP_INCLUDED
#define LIBLAS_DIMENSION_HPP_INCLUDED

#include <string>

// boost
#include <boost/property_tree/ptree.hpp>
#include <boost/cstdint.hpp>

#include <libpc/export.hpp>


namespace libpc
{


/// A Dimension consists of a name, a datatype, and (for the bitfield datatype), the number
/// of bits the dimension requires.
///
/// When a dimension is added to a Schema, it also gets two more properties: the position (index)
/// of this dimension in the schema's list of dimensions, and the byte offset where the dimension
/// is stored in the PointBuffer's raw bytes
class LIBPC_DLL Dimension
{
public:
    enum DataType
    {
        int8_t,
        uint8_t,
        int16_t,
        uint16_t,
        int32_t,
        uint32_t,
        int64_t,
        uint64_t,
        float_t,       // 32 bits
        double_t      // 64 bits
    };

public:
    Dimension(std::string const& name, DataType type);
    Dimension& operator=(Dimension const& rhs);
    Dimension(Dimension const& other);

    bool operator==(const Dimension& other) const;

    virtual ~Dimension() {}

    inline std::string const& getName() const
    {
        return m_name;
    }

    DataType getDataType() const
    {
        return m_dataType;
    }

    static std::string getDataTypeName(DataType);
    static DataType getDataTypeFromString(const std::string&);
    static std::size_t getDataTypeSize(DataType);
    static bool getDataTypeIsNumeric(DataType);
    static bool getDataTypeIsSigned(DataType);
    static bool getDataTypeIsInteger(DataType);

    /// bytes, physical/serialisation size of record
    // for bitfields, this will be rounded up to the next largest byte
    std::size_t getByteSize() const
    {
       return m_byteSize;
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

    inline std::string getDescription() const
    {
        return m_description;
    }
    inline void setDescription(std::string const& v)
    {
        m_description = v;
    }

    /// Is this dimension a numeric dimension.  Dimensions with IsNumeric == false
    /// are considered generic bit/byte fields
    inline bool isNumeric() const
    {
        return getDataTypeIsNumeric(m_dataType);
    }

    /// Does this dimension have a sign?  Only applicable to dimensions with
    /// IsNumeric == true.
    inline bool isSigned() const
    {
        return getDataTypeIsSigned(m_dataType);
    }

    /// Does this dimension interpret to an integer?  Only applicable to dimensions
    /// with IsNumeric == true.
    inline bool isInteger() const
    {
        return getDataTypeIsInteger(m_dataType);
    }

    /// The minimum value of this dimension as a double
    inline double getMinimum() const
    {
        return m_min;
    }
    inline void setMinimum(double min)
    {
        m_min = min;
    }

    /// The maximum value of this dimension as a double
    inline double getMaximum() const
    {
        return m_max;
    }
    inline void setMaximum(double max)
    {
        m_max = max;
    }

    /// The index position of the index.  In a standard ePointFormat0
    /// data record, the X dimension would have a position of 0, while
    /// the Y dimension would have a position of 1, for example.
    inline std::size_t getPosition() const
    {
        return m_position;
    }

    inline void setPosition(std::size_t v)
    {
        m_position = v;
    }

    /// The scaling value for this dimension as a double.  This should
    /// be positive or negative powers of ten.
    inline double getNumericScale() const
    {
        return m_numericScale;
    }
    inline void setNumericScale(double v)
    {
        m_numericScale = v;
    }

    /// The offset value for this dimension.  Usually zero, but it
    /// can be set to any value in combination with the scale to
    /// allow for more expressive ranges.
    /// (this is not called just "Offset" anymore, since that term also
    /// means "what bit/byte position a field is located at")
    inline double getNumericOffset() const
    {
        return m_numericOffset;
    }
    inline void setNumericOffset(double v)
    {
        m_numericOffset = v;
    }

    /// If true, this dimension uses the numeric scale/offset values
    inline bool isFinitePrecision() const
    {
        return m_precise;
    }
    inline void isFinitePrecision(bool v)
    {
        m_precise = v;
    }

    boost::property_tree::ptree GetPTree() const;

private:
    DataType m_dataType;
    std::string m_name;
    std::size_t m_byteSize;
    std::size_t m_byteOffset;
    std::string m_description;
    double m_min;
    double m_max;
    std::size_t m_position;
    bool m_precise;
    double m_numericScale;
    double m_numericOffset;
};



LIBPC_DLL std::ostream& operator<<(std::ostream& os, libpc::Dimension const& d);


} // namespace libpc

#endif // LIBLAS_DIMENSION_HPP_INCLUDED
