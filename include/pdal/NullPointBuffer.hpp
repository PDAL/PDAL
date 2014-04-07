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
#pragma once

#include <pdal/PointBuffer.hpp>

namespace pdal
{

class PDAL_DLL NullPointBuffer : public PointBuffer
{
public:
    NullPointBuffer();
    
    virtual const Bounds<double>& getSpatialBounds() const;
    virtual void setSpatialBounds(const Bounds<double>& bounds);
    virtual boost::uint32_t getNumPoints() const
        { return 0; }
    virtual void setNumPoints(boost::uint32_t v)
        { (void)v; }
    virtual boost::uint32_t getCapacity() const
        { return std::numeric_limits<boost::uint32_t>::max(); }
    virtual pointbuffer::PointBufferByteSize getBufferByteCapacity() const
    {
        return std::numeric_limits<pointbuffer::PointBufferByteSize>::max();
    }
    virtual void getData(boost::uint8_t** data, boost::uint64_t* size) const;
    virtual void setData(boost::uint8_t* data, boost::uint32_t pointIndex);
    virtual void setDataStride(boost::uint8_t* data, boost::uint32_t pointIndex,
        boost::uint32_t byteCount);
    virtual void reset(Schema const& new_schema);
    virtual void resize(boost::uint32_t const& capacity, bool bExact=false);
    virtual PointBuffer* pack(bool bRemoveIgnoredDimensions = true) const;
    virtual PointBuffer* flipOrientation() const;
    virtual boost::property_tree::ptree toPTree() const;
    virtual std::ostream& toRST(std::ostream& os) const;
    virtual pdal::Bounds<double> calculateBounds(bool bis3d=true) const;
    virtual double applyScaling(Dimension const& d,
        std::size_t pointIndex) const;

protected:
    virtual std::string printDimension(Dimension const& dimension,
        boost::uint32_t index) const;
};

} // namespace pdal

