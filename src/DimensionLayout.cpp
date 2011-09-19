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

#include <pdal/DimensionLayout.hpp>

#include <boost/property_tree/json_parser.hpp>


namespace pdal
{


DimensionLayout::DimensionLayout(const Dimension& dimension)
    : m_dimension(dimension)
    , m_byteOffset(0)
    , m_position(0)
    , m_isValid(false)
{
}

/// copy constructor
DimensionLayout::DimensionLayout(DimensionLayout const& other) 
    : m_dimension(other.m_dimension)
    , m_byteOffset(other.m_byteOffset)
    , m_position(other.m_position)
    , m_isValid(other.m_isValid)
{
}

/// assignment operator
DimensionLayout& DimensionLayout::operator=(DimensionLayout const& rhs)
{
    if (&rhs != this)
    {
        m_dimension = rhs.m_dimension;
        m_position = rhs.m_position;
        m_byteOffset = rhs.m_byteOffset;
        m_isValid = rhs.m_isValid;
    }

    return *this;
}


bool DimensionLayout::operator==(const DimensionLayout& other) const
{
    if (m_dimension == other.m_dimension)
    {
        assert(m_position == other.m_position);
        assert(m_byteOffset == other.m_byteOffset);
        assert(m_isValid == other.m_isValid);
        return true;
    }

    return false;
}


bool DimensionLayout::operator!=(const DimensionLayout& other) const
{
  return !(*this==other);
}


boost::property_tree::ptree DimensionLayout::toPTree() const
{
    boost::property_tree::ptree tree;

    tree.add_child("dimension", getDimension().toPTree());
    tree.add("byteoffset", getByteOffset());
    tree.add("position", getPosition());
    tree.add("isValid", isValid());

    return tree;
}


void DimensionLayout::dump() const
{
    std::cout << *this;
}


std::ostream& operator<<(std::ostream& os, pdal::DimensionLayout const& layout)
{
    boost::property_tree::ptree tree = layout.toPTree();

    boost::property_tree::write_json(os, tree);

    return os;
}


} // namespace pdal
