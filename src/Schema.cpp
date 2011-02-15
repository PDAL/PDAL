/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS Schema implementation for C++ libLAS
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

#include <libpc/Schema.hpp>
#include <libpc/Utils.hpp>

// boost
#include <boost/concept_check.hpp>
#include <boost/foreach.hpp>

using namespace boost;

namespace libpc
{


Schema::Schema()
    : m_byteSize(0)
{
    return;
}


/// copy constructor
Schema::Schema(Schema const& other) 
    : m_dimensions(other.m_dimensions)
    , m_byteSize(other.m_byteSize)
{
}


// assignment constructor
Schema& Schema::operator=(Schema const& rhs)
{
    if (&rhs != this)
    {
        m_dimensions = rhs.m_dimensions;
        m_byteSize = rhs.m_byteSize;
    }

    return *this;
}


bool Schema::operator==(const Schema& other) const
{
    if (m_byteSize == other.m_byteSize &&
        m_dimensions == other.m_dimensions)
    {
        return true;
    }

    return false;
}


property_tree::ptree Schema::getPTree() const
{
    using property_tree::ptree;
    ptree pt;

    for (DimensionsCIter iter = m_dimensions.cbegin(); iter != m_dimensions.cend(); ++iter)
    {
        pt.add_child("LASSchema.dimensions.dimension", (*iter).GetPTree());
    }

    return pt;
}


void Schema::calculateSizes()
{
    // to make life easy, for now we are going to assume that each Dimension 
    // is byte-aligned and occupies an integral number of bytes

    std::size_t offset = 0;

    int i=0;
    for (DimensionsIter iter = m_dimensions.begin(); iter != m_dimensions.end(); ++iter)
    {
        Dimension& t = *iter;

        t.setByteOffset(offset);

        offset += t.getByteSize();

        t.setPosition(i);
        ++i;
    }

    m_byteSize = offset;

    return;
}


std::size_t Schema::getByteSize() const
{
    return m_byteSize;
}


void Schema::addDimension(Dimension const& dim)
{
    // BUG: assert not already added

    m_dimensions.push_back(dim);

    // Update all of our sizes
    calculateSizes();

    return;
}


std::vector<std::string> Schema::getDimensionNames() const
{
    std::vector<std::string> output;

    for (DimensionsCIter iter = m_dimensions.cbegin(); iter != m_dimensions.cend(); ++iter)
    {
        output.push_back(iter->getName());
    }

    return output;
}


bool Schema::findDimensionIndex(const std::string& name, std::size_t& index) const
{
    for (DimensionsCIter iter = m_dimensions.cbegin(); iter != m_dimensions.cend(); ++iter)
    {
        if (iter->getName() == name)
        {
            index = iter->getPosition();
            return true;
        }
        ++index;
    }
    return false;
}


bool Schema::hasDimension(const std::string& name) const
{
    for (DimensionsCIter iter = m_dimensions.cbegin(); iter != m_dimensions.cend(); ++iter)
    {
        if (iter->getName() == name)
        {
            return true;
        }
    }
    return false;
}


std::ostream& operator<<(std::ostream& os, Schema const& schema)
{
    using property_tree::ptree;
    ptree tree = schema.getPTree();

    os << "---------------------------------------------------------" << std::endl;
    os << "  Schema Summary" << std::endl;
    os << "---------------------------------------------------------" << std::endl;

    ptree::const_iterator i;

    //ptree dims = tree.get_child("LASSchema.dimensions");
    ///////////os << "  Point Format ID:             " << tree.get<std::string>("LASSchema.formatid") << std::endl;
    os << "  Number of dimensions:        " << schema.getDimensions().size() << std::endl;
    os << "  Size in bytes:               " << schema.getByteSize() << std::endl;

    os << std::endl;
    os << "  Dimensions" << std::endl;
    os << "---------------------------------------------------------" << std::endl;

    os << "  ";

    const Schema::Dimensions& dimensions = schema.getDimensions();
    for (Schema::DimensionsCIter iter = dimensions.cbegin(); iter != dimensions.cend(); ++iter)
    {
        os << *iter;
        os << "  ";
    }


    os << std::endl;

    return os;
}


} // namespace libpc
