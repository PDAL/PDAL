/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
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

#include <libpc/Header.hpp>

#include <iostream>

using std::endl;

namespace libpc
{


Header::Header()
    : m_numPoints(0)
    , m_pointCountType(PointCount_Fixed)
{
    return;
}


Header::Header(const Header& other)
{
    this->m_numPoints = other.m_numPoints;
    this->m_pointCountType = other.m_pointCountType;
    this->m_schema = other.m_schema;
    this->m_bounds = other.m_bounds;
    this->m_spatialReference = other.m_spatialReference;
    return;
}


Header::~Header()
{
    return;
}


const Bounds<double>& Header::getBounds() const
{
    return m_bounds;
}


void Header::setBounds(const Bounds<double>& bounds)
{
    m_bounds = bounds;
}


const Schema& Header::getSchema() const
{
    return m_schema;
}


Schema& Header::getSchema()
{
    return m_schema;
}


void Header::setSchema(const Schema& schema)
{
    m_schema = schema;
}


boost::uint64_t Header::getNumPoints() const
{
    return m_numPoints;
}


void Header::setNumPoints(boost::uint64_t numPoints)
{
    m_numPoints = numPoints;
}


PointCountType Header::getPointCountType() const
{
    return m_pointCountType;
}


void Header::setPointCountType(PointCountType pointCountType)
{
    m_pointCountType = pointCountType;
}


const SpatialReference& Header::getSpatialReference() const
{
    return m_spatialReference;
}


void Header::setSpatialReference(const SpatialReference& spatialReference)
{
    m_spatialReference = spatialReference;
}


const Metadata::Array& Header::getMetadata() const
{
    return m_metadataArray;
}


Metadata::Array& Header::getMetadata()
{
    return m_metadataArray;
}


void Header::dump() const
{
    std::cout << *this;
}


std::ostream& operator<<(std::ostream& ostr, const Header& header)
{
    ostr << "  Num points: " << header.getNumPoints() << endl;

    ostr << "  Bounds:" << endl;
    ostr << "    " << header.getBounds() << endl;

    ostr << "  Schema: " << endl;
    ostr << "    Num dims: " << header.getSchema().getDimensions().size() << endl;
//    ostr << "    Size in bytes: " << header.getSchema().getByteSize() << endl;

    ostr << "  Spatial Reference:" << endl;
    ostr << "    WKT: " << header.getSpatialReference().getWKT() << endl;

    return ostr;
}


} // namespace libpc
