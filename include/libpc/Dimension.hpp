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

#include <libpc/version.hpp>
#include <libpc/export.hpp>


namespace libpc
{


/// Dimension definition
class LIBPC_DLL Dimension
{
public:
    Dimension(std::string const& name, std::size_t size_in_bits);
    Dimension& operator=(Dimension const& rhs);
    Dimension(Dimension const& other);

    virtual ~Dimension() {}

    inline std::string const& GetName() const
    {
        return m_name;
    }

    /// bits, total logical size of point record, including any custom
    /// dimensions
    inline std::size_t GetBitSize() const
    {
        return m_bit_size;
    }

    /// bytes, physical/serialisation size of record
    std::size_t GetByteSize() const;

    /// The byte location to start reading/writing
    /// point data from in a composited schema.  liblas::Schema
    /// will set these values for you when liblas::Dimension are
    /// added to the liblas::Schema.
    inline std::size_t GetByteOffset() const
    {
        return m_byte_offset;
    }

    inline void SetByteOffset(std::size_t v)
    {
        m_byte_offset = v;
    }

    /// The bit location within the byte to start reading data.  liblas::Schema
    /// will set these values for you when liblas::Dimension are
    /// added to the liblas::Schema.  This value will be 0 for dimensions
    /// that are composed of entire bytes.
    inline std::size_t GetBitOffset() const
    {
        return m_bit_offset;
    }

    inline void SetBitOffset(std::size_t v)
    {
        m_bit_offset = v;
    }

    /// Is this dimension required by PointFormatName
    inline bool IsRequired() const
    {
        return m_required;
    }
    inline void IsRequired(bool v)
    {
        m_required = v;
    }

    /// Is this dimension being used.  A dimension with
    /// IsActive false may exist as a placeholder in PointFormatName-specified
    /// dimensions, but have their IsActive flag set to false.  In this
    /// case, those values may be disregarded.
    inline bool IsActive() const
    {
        return m_active;
    }
    inline void IsActive(bool v)
    {
        m_active = v;
    }

    inline std::string GetDescription() const
    {
        return m_description;
    }
    inline void SetDescription(std::string const& v)
    {
        m_description = v;
    }

    /// Is this dimension a numeric dimension.  Dimensions with IsNumeric == false
    /// are considered generic bit/byte fields/
    inline bool IsNumeric() const
    {
        return m_numeric ;
    }
    inline void IsNumeric(bool v)
    {
        m_numeric = v;
    }

    /// Does this dimension have a sign?  Only applicable to dimensions with
    /// IsNumeric == true.
    inline bool IsSigned() const
    {
        return m_signed;
    }
    inline void IsSigned(bool v)
    {
        m_signed = v;
    }

    /// Does this dimension interpret to an integer?  Only applicable to dimensions
    /// with IsNumeric == true.
    inline bool IsInteger() const
    {
        return m_integer;
    }
    inline void IsInteger(bool v)
    {
        m_integer = v;
    }

    /// The minimum value of this dimension as a double
    inline double GetMinimum() const
    {
        return m_min;
    }
    inline void SetMinimum(double min)
    {
        m_min = min;
    }

    /// The maximum value of this dimension as a double
    inline double GetMaximum() const
    {
        return m_max;
    }
    inline void SetMaximum(double max)
    {
        m_max = max;
    }

    /// The index position of the index.  In a standard ePointFormat0
    /// data record, the X dimension would have a position of 0, while
    /// the Y dimension would have a position of 1, for example.
    inline boost::uint32_t GetPosition() const
    {
        return m_position;
    }
    inline void SetPosition(boost::uint32_t v)
    {
        m_position = v;
    }

    /// The scaling value for this dimension as a double.  This should
    /// be positive or negative powers of ten.
    inline double GetScale() const
    {
        return m_scale;
    }
    inline void SetScale(double v)
    {
        m_scale = v;
    }

    /// The offset value for this dimension.  Usually zero, but it
    /// can be set to any value in combination with the scale to
    /// allow for more expressive ranges.
    inline double GetOffset() const
    {
        return m_offset;
    }
    inline void SetOffset(double v)
    {
        m_offset = v;
    }

    /// If true, this dimension uses scale/offset values
    inline bool IsFinitePrecision() const
    {
        return m_precise;
    }
    inline void IsFinitePrecision(bool v)
    {
        m_precise = v;
    }

    inline bool operator < (Dimension const& dim) const
    {
        return m_position < dim.m_position;
    }
    inline bool operator > (Dimension const& dim) const
    {
        return m_position > dim.m_position;
    }

    boost::property_tree::ptree GetPTree() const;

private:
    std::string m_name;
    std::size_t m_bit_size;
    bool m_required;
    bool m_active;
    std::string m_description;
    double m_min;
    double m_max;
    bool m_numeric;
    bool m_signed;
    bool m_integer;
    boost::uint32_t m_position;
    double m_scale;
    bool m_precise;
    double m_offset;
    std::size_t m_byte_offset;
    std::size_t m_bit_offset;
};


struct SetRequired
{
    SetRequired(bool req) : req_(req) {}

    void operator()(Dimension& e)
    {
        e.IsRequired(req_);
    }

private:
    bool req_;
};


struct SetActive
{
    SetActive(bool req) : req_(req) {}

    void operator()(Dimension& e)
    {
        e.IsActive(req_);
    }

private:
    bool req_;
};



LIBPC_DLL std::ostream& operator<<(std::ostream& os, libpc::Dimension const& d);


} // namespace libpc

#endif // LIBLAS_DIMENSION_HPP_INCLUDED
