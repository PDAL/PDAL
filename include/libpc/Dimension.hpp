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

#ifndef LIBPC_DIMENSION_HPP_INCLUDED
#define LIBPC_DIMENSION_HPP_INCLUDED

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
    enum Field
    {
        Field_INVALID = 0,
        Field_X,
        Field_Y,
        Field_Z,
        Field_Intensity,
        Field_ReturnNumber,
        Field_NumberOfReturns,
        Field_ScanDirectionFlag,
        Field_EdgeOfFlightLine,
        Field_Classification,
        Field_ScanAngleRank,
        Field_UserData,
        Field_PointSourceId,
        Field_Time,
        Field_Red,
        Field_Green,
        Field_Blue,
        Field_WavePacketDescriptorIndex,
        Field_WaveformDataOffset,
        Field_ReturnPointWaveformLocation,
        Field_WaveformXt,
        Field_WaveformYt,
        Field_WaveformZt,
        // ...

        // add more here
        Field_User1 = 512,
        Field_User2,
        Field_User3,
        Field_User4,
        Field_User5,
        Field_User6,
        Field_User7,
        Field_User8,
        Field_User9,
        // ...
        // feel free to use your own int here

        Field_LAST = 1023
    };

    enum DataType
    {
        Int8,
        Uint8,
        Int16,
        Uint16,
        Int32,
        Uint32,
        Int64,
        Uint64,
        Float,       // 32 bits
        Double       // 64 bits
    };

public:
    Dimension(Field field, DataType type);
    Dimension& operator=(Dimension const& rhs);
    Dimension(Dimension const& other);

    bool operator==(const Dimension& other) const;
    bool operator!=(const Dimension& other) const;

    std::string const& getFieldName() const;

    Field getField() const
    {
        return m_field;
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
    static std::string const& getFieldName(Field);

    /// bytes, physical/serialisation size of record
    // for bitfields, this will be rounded up to the next largest byte
    std::size_t getByteSize() const
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

    template<class T>
    double getNumericValue(T x) const
    {
        return (double)x * m_numericScale + m_numericOffset;
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
    Field m_field;
    std::size_t m_byteSize;
    std::string m_description;
    double m_min;
    double m_max;
    bool m_precise;
    double m_numericScale;
    double m_numericOffset;

    static void initFieldNames();
    static bool s_fieldNamesValid;
    static std::string s_fieldNames[Field_LAST];
};


LIBPC_DLL std::ostream& operator<<(std::ostream& os, libpc::Dimension const& d);


} // namespace libpc

#endif // LIBPC_DIMENSION_HPP_INCLUDED
