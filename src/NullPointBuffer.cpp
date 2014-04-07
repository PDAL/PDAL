/******************************************************************************
* Copyright (c) 2014, Andrew Bell
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

#include <pdal/NullPointBuffer.hpp>

namespace pdal
{

NullPointBuffer::NullPointBuffer() : PointBuffer( Schema() )
{}


const Bounds<double>& NullPointBuffer::getSpatialBounds() const
{
    return m_bounds;
}


void NullPointBuffer::setSpatialBounds(const Bounds<double>& bounds)
{
    (void)bounds;
}


void NullPointBuffer::getData(boost::uint8_t** data,
    boost::uint64_t* array_size) const
{
    *array_size = 0;
    *data = NULL;
}


void NullPointBuffer::setData(boost::uint8_t* data, boost::uint32_t pointIndex)
{
    (void)data;
    (void)pointIndex;
}


void NullPointBuffer::setDataStride(boost::uint8_t* data,
    boost::uint32_t pointIndex, boost::uint32_t byteCount)
{
    (void)data;
    (void)pointIndex;
    (void)byteCount;
}


void NullPointBuffer::reset(Schema const& new_schema)
{
    (void)new_schema;
}


void NullPointBuffer::resize(boost::uint32_t const& capacity, bool bExact)
{
    (void)capacity;
    (void)bExact;
}


PointBuffer* NullPointBuffer::pack(bool bRemoveIgnoredDimensions) const
{
    return const_cast<NullPointBuffer *>(this);
}


PointBuffer* NullPointBuffer::flipOrientation() const
{
    return const_cast<NullPointBuffer *>(this);
}


boost::property_tree::ptree NullPointBuffer::toPTree() const
{
    boost::property_tree::ptree tree;
    return tree;
}


std::ostream& NullPointBuffer::toRST(std::ostream& os) const
{
    return os;
}


pdal::Bounds<double> NullPointBuffer::calculateBounds(bool is3d) const
{
    (void)is3d;
    return Bounds<double>();
}


double NullPointBuffer::applyScaling(Dimension const& d,
    std::size_t pointIndex) const
{
    (void)d;
    (void)pointIndex;
    return 0.0;
}


std::ostream& operator<<(std::ostream& ostr, const NullPointBuffer& pointBuffer)
{
    ostr << "Contains no points" << std::endl;
    return ostr;
}


std::string NullPointBuffer::printDimension(Dimension const& dimension,
    boost::uint32_t index) const
{
    (void)dimension;
    (void)index;
    return std::string();
}


} // namespace pdal
