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

#include <pdal/pdal.hpp>
#include <pdal/Utils.hpp>

#include <boost/property_tree/ptree.hpp>


namespace pdal
{


/// A Dimension consists of a name and a datatype.
///
/// When a dimension is added to a Schema, it also gets two more properties: the position (index)
/// of this dimension in the schema's list of dimensions, and the byte offset where the dimension
/// is stored in the PointBuffer's raw bytes
class PDAL_DLL Dimension
{
public:
    enum Id
    {
        //
        // common field types: 0..999
        // 
        Id_X_i32 = 0,
        Id_Y_i32,
        Id_Z_i32,
        Id_X_f64,
        Id_Y_f64,
        Id_Z_f64,

        Id_Red_u8,
        Id_Green_u8,
        Id_Blue_u8,
        Id_Red_u16,
        Id_Green_u16,
        Id_Blue_u16,
    
        Id_Time_u64,

        //
        // LAS: 1000..1999
        //
        Id_Las_Intensity = 1000,
        Id_Las_ReturnNumber,
        Id_Las_NumberOfReturns,
        Id_Las_ScanDirectionFlag,
        Id_Las_EdgeOfFlightLine,
        Id_Las_Classification,
        Id_Las_ScanAngleRank,
        Id_Las_UserData,
        Id_Las_PointSourceId,
        Id_Las_WavePacketDescriptorIndex,
        Id_Las_WaveformDataOffset,
        Id_Las_ReturnPointWaveformLocation,
        Id_Las_WaveformXt,
        Id_Las_WaveformYt,
        Id_Las_WaveformZt,
        Id_Las_Time,

        //
        // terrasolid: 2000..2999
        // 
        Id_TerraSolid_Alpha = 2000,
        Id_TerraSolid_Classification,
        Id_TerraSolid_PointSourceId_u8,
        Id_TerraSolid_PointSourceId_u16,
        Id_TerraSolid_ReturnNumber_u8,
        Id_TerraSolid_ReturnNumber_u16,
        Id_TerraSolid_Flag,
        Id_TerraSolid_Mark,
        Id_TerraSolid_Intensity,
        Id_TerraSolid_Time,

        //
        // chipper stuff: 3000..3999
        // 
        Id_Chipper_1 = 3000,
        Id_Chipper_2,

        //
        // qfit: 4000..4999
        // 
        Id_Qfit_StartPulse = 4000,
        Id_Qfit_ReflectedPulse,
        Id_Qfit_ScanAngleRank,
        Id_Qfit_Pitch,
        Id_Qfit_Roll,
        Id_Qfit_Time,
        Id_Qfit_PassiveSignal,
        Id_Qfit_PassiveX,
        Id_Qfit_PassiveY,
        Id_Qfit_PassiveZ,
        Id_Qfit_GpsTime,
        Id_Qfit_PDOP,
        Id_Qfit_PulseWidth,

        // user fields are 100,000..199,999

        Id_Undefined = 200000
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
        Pointer,    // stored as 64 bits, even on a 32-bit box
        Float,       // 32 bits
        Double,       // 64 bits
        Undefined
    };

    enum Flags
    {
        IsAdded   = 0x1,
        IsRead    = 0x2,
        IsWritten = 0x4,
    };

/// \name Constructors
    Dimension(Id id); // will use table to lookup datatype, description, etc
    Dimension(Id id, DataType datatype, std::string name, std::string description=std::string("")); // for dimensions not in the master table
    Dimension(Dimension const& other);

    Dimension& operator=(Dimension const& rhs);

/// \name Equality
    bool operator==(const Dimension& other) const;
    bool operator!=(const Dimension& other) const;

/// \name Data Access
    std::string const& getName() const;

    Id getId() const
    {
        return m_id;
    }

    boost::uint32_t getFlags() const { return m_flags; }
    void setFlags(boost::uint32_t flags) { m_flags = flags; }

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

    /// \return Number of bytes required to serialize this dimension 
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

    /*!
        \return Is this dimension a numeric dimension?  
        \verbatim embed:rst 
        .. note::
            
            Dimensions with isNumeric == false are considered generic byte fields.
        \endverbatim
    */
    inline bool isNumeric() const
    {
        return getDataTypeIsNumeric(m_dataType);
    }

    /*!
        \return Does this dimension a sign?  
        \verbatim embed:rst 
        .. note::
            
            Only applicable to dimensions with isNumeric() == true.
        \endverbatim
    */    
    inline bool isSigned() const
    {
        return getDataTypeIsSigned(m_dataType);
    }

    /*!
        \return Does this dimension interpret to an integer?
        \verbatim embed:rst 
        .. note::
            
            Only applicable to dimensions with isNumeric() == true.
        \endverbatim
    */      
    inline bool isInteger() const
    {
        return getDataTypeIsInteger(m_dataType);
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
        // If you set a scale that isn't 0, you just made the dimension 
        // finite precision by default.
        if ( !Utils::compare_approx(v, 0.0, (std::numeric_limits<double>::min)()))
        {
            m_precise = true;
        }        

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
        if ( !Utils::compare_approx(v, 0.0, (std::numeric_limits<double>::min)()))
        {
            m_precise = true;
        }       
        m_numericOffset = v;
    }

    /// Applies the scale and offset values from the dimension to a the given value
    /// \param v The value to scale with the Dimension's NumericOffset and NumericScale values
    /// \return \b v scaled by getNumericOffset() and getNumericScale() values.
    template<class T>
    double applyScaling(T v) const
    {
        return static_cast<double>(v) * m_numericScale + m_numericOffset;
    }

    /// Removes the scale and offset values from an imprecise double value
    /// \param v The value to descale with the Dimension's NumericScale and NumericOffset values
    /// \return a value that has been descaled by the Dimension's NumericOffset and NumericScale values
    template<class T>
    T removeScaling(double v) const
    {
        T output = static_cast<T>(Utils::sround((v - m_numericOffset)/ m_numericScale));
        return output;
    }

    /// Gets whether this Dimension uses the numeric scale/offset values
    inline bool isFinitePrecision() const
    {
        return m_precise;
    }
    /// Sets whether or not this Dimension uses numerica scale/offset values
    inline void isFinitePrecision(bool v)
    {
        m_precise = v;
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

/// \name Summary and serialization
    /// Outputs a string-based boost::property_tree::ptree representation 
    /// of the Dimension instance
    boost::property_tree::ptree toPTree() const;

    /// Outputs a string representation of the Dimension instance to std::cout
    void dump() const;

private:
    DataType m_dataType;
    Id m_id;
    std::string m_name;
    boost::uint32_t m_flags;
    EndianType m_endian;
    std::size_t m_byteSize;
    std::string m_description;
    double m_min;
    double m_max;
    bool m_precise;
    double m_numericScale;
    double m_numericOffset;
};


PDAL_DLL std::ostream& operator<<(std::ostream& os, pdal::Dimension const& d);

} // namespace pdal

#endif // PDAL_DIMENSION_HPP_INCLUDED
