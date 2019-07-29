/******************************************************************************
* Copyright (c) 2016, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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

#pragma once

#include <array>
#include <iterator>

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/Utils.hpp>

namespace pdal
{

class PDAL_DLL MemoryViewReader : public Reader, public Streamable
{
public:
    enum class Order
    {
        RowMajor,
        ColumnMajor,
    };

    struct Shape
    {
        Shape() : m_shape{ 0, 0, 0 }
        {}

        Shape(size_t depth, size_t rows, size_t columns) :
            m_shape({depth, rows, columns})
        {}

        Shape(const std::array<size_t, 3>& s) : m_shape(s)
        {}

        size_t depth() const
        {
            return m_shape[0];
        }
        size_t rows() const
        {
            return m_shape[1];
        }
        size_t columns() const
        {
            return m_shape[2];
        }

        bool valid() const
        {
            return m_shape[0] && m_shape[1] && m_shape[2];
        }

        std::array<size_t, 3> m_shape;

        friend std::istream& operator>>(std::istream& in, Shape& shape);
        friend const std::ostream& operator<<(const std::ostream& out,
            Shape& shape);
    };

    struct Field
    {
        std::string m_name;
        Dimension::Type m_type;
        size_t m_offset;
    };
    using PointIncrementer = std::function<char *(PointId)>; 

private:
    struct FullField : public Field
    {
        FullField(const Field& f) : Field(f), m_id(Dimension::Id::Unknown)
        {}

        Dimension::Id m_id;
    };


public:
    std::string getName() const;

    MemoryViewReader();

    /**
      Push a data field into the structure of ordered fields.

      \param f  Field to add.
    */
    void pushField(const Field& f);

    /**
      Set a function that handles modifying the memory location of
      subsequent points.
      
      \param inc  A function that is called by MemoryViewReader with the
        current point ID.  The function should return the base pointer
        of the point, or nullptr if there are no more points to read.
    */
    void setIncrementer(PointIncrementer inc)
    {
        m_incrementer = inc;
    }

private:
    /**
    */
    virtual void addArgs(ProgramArgs& args)
    {
        args.add("order", "Order of synthetic X/Y/Z values "
            "('row' or 'column').", m_order, Order::RowMajor);
        args.add("shape", "Shape of memory (depth, rows, columns).", m_shape);
    }

    /**
      Add dimensions from the input to the layout.

      \param layout  Layout to which the dimenions are added.
    */
    virtual void addDimensions(PointLayoutPtr layout);

    /**
      Initialize the stage.
    */
    virtual void initialize();

    /**
      Set state based on the stage being prepared.
    */
    virtual void prepared(PointTableRef);

    /**
      Make MemoryViewReader ready for reading.
    */
    virtual void ready(PointTableRef);

    /**
      Read up to numPts points into the \ref view.

      \param view  PointView in which to insert point data.
      \param numPts  Maximum number of points to read.
      \return  Number of points read.
    */
    virtual point_count_t read(PointViewPtr view, point_count_t numPts);

    /**
      Read a single point from the input.

      \param point  Reference to point to fill with data.
      \return  False if no point could be read.
    */
    virtual bool processOne(PointRef& point);

private:
    PointIncrementer m_incrementer;
    std::vector<FullField> m_fields;
    bool m_prepared;
    PointId m_index;
    Shape m_shape;
    Order m_order;
    size_t m_xIter;
    size_t m_yIter;
    size_t m_zIter;
    size_t m_xDiv;
    size_t m_yDiv;
    size_t m_zDiv;
};

inline std::istream& operator>>(std::istream& in,
    MemoryViewReader::Order& order)
{
    std::string s(std::istreambuf_iterator<char>(in), {});

    s = Utils::toupper(s);
    if (s == "ROW")
        order = MemoryViewReader::Order::RowMajor;
    else if (s == "COLUMN")
        order = MemoryViewReader::Order::ColumnMajor;
    else
        throw pdal_error("Invalid value for option 'order'.  Must be 'row'"
            " or 'column'.");
    return in;
}

inline std::ostream& operator<<(std::ostream& out,
    const MemoryViewReader::Order& order)
{
    if (order == MemoryViewReader::Order::RowMajor)
        out << "row";
    else
        out << "column";
    return out;
}

inline std::istream& operator>>(std::istream& in,
    MemoryViewReader::Shape& shape)
{
    std::string s(std::istreambuf_iterator<char>(in), {});
    StringList values = Utils::split2(s, ',');
    if (!values.size())
        throw pdal_error("Shape must be specified as three integers: "
            "'depth, rows, columns'.");
    Utils::trim(values[0]);
    Utils::trim(values[1]);
    Utils::trim(values[2]);

    size_t len;
    size_t depth = std::stol(values[0], &len);
    if (len != values[0].size())
        throw pdal_error("Invalid depth value in shape: '" + values[0] + "'.");
    size_t rows = std::stol(values[1], &len);
    if (len != values[1].size())
        throw pdal_error("Invalid rows value in shape: '" + values[1] + "'.");
    size_t cols = std::stol(values[2], &len);
    if (len != values[2].size())
        throw pdal_error("Invalid rows value in shape: '" + values[2] + "'.");

    shape = {{ depth, rows, cols }};

    return in;
}

inline std::ostream& operator<<(std::ostream& out,
    const MemoryViewReader::Shape& shape)
{
    out << shape.depth() << ", " << shape.rows() << ", " << shape.columns();
    return out;
}

} // namespace pdal
