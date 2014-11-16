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

#include <iomanip>

#include <pdal/PointBuffer.hpp>
#include <pdal/PointBufferIter.hpp>

#include <boost/lexical_cast.hpp>

namespace pdal
{

PointBufferIter PointBuffer::begin()
{
    return PointBufferIter(this, 0);
}


PointBufferIter PointBuffer::end()
{
    return PointBufferIter(this, size());
}


BOX3D PointBuffer::calculateBounds(bool is3d) const
{
    BOX3D output;

    bool first = true;
    for (PointId idx = 0; idx < size(); idx++)
    {
        double x = getFieldAs<double>(Dimension::Id::X, idx);
        double y = getFieldAs<double>(Dimension::Id::Y, idx);
        double z = getFieldAs<double>(Dimension::Id::Z, idx);

        if (is3d)
        {
            if (first)
            {
                output = BOX3D(x, y, z, x, y, z);
                first = false;
            }
            output.grow(x,y,z);
        }
        else
        {
            if (first)
            {
                output = BOX3D(x, y, x, y);
                first = false;
            }
            output.grow(x, y);
        }
    }
    return output;
}


BOX3D PointBuffer::calculateBounds(const PointBufferSet& set, bool is3d)
{
    BOX3D out;
    for (auto iter = set.begin(); iter != set.end(); ++iter)
    {
        PointBufferPtr buf = *iter;
        out.grow(buf->calculateBounds(is3d));
    }
    return out;
}


void PointBuffer::dump(std::ostream& ostr) const
{
    using std::endl;
    const Dimension::IdList& dims = m_context.dims();

    point_count_t numPoints = size();
    ostr << "Contains " << numPoints << "  points" << endl;
    for (PointId idx = 0; idx < numPoints; idx++)
    {
        ostr << "Point: " << idx << endl;

        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            Dimension::Id::Enum d = *di;
            Dimension::Detail *dd = m_context.dimDetail(d);
            ostr << Dimension::name(d) << " (" <<
                Dimension::interpretationName(dd->type()) << ") : ";

            switch (dd->type())
            {
            case Dimension::Type::Signed8:
                {
                    ostr << (int)(getFieldInternal<int8_t>(d, idx));
                    break;
                }
            case Dimension::Type::Signed16:
                {
                    ostr << getFieldInternal<int16_t>(d, idx);
                    break;
                }
            case Dimension::Type::Signed32:
                {
                    ostr << getFieldInternal<int32_t>(d, idx);
                    break;
                }
            case Dimension::Type::Signed64:
                {
                    ostr << getFieldInternal<int64_t>(d, idx);
                    break;
                }
            case Dimension::Type::Unsigned8:
                {
                    ostr << (unsigned)(getFieldInternal<uint8_t>(d, idx));
                    break;
                }
            case Dimension::Type::Unsigned16:
                {
                    ostr << getFieldInternal<uint16_t>(d, idx);
                    break;
                }
            case Dimension::Type::Unsigned32:
                {
                    ostr << getFieldInternal<uint32_t>(d, idx);
                    break;
                }
            case Dimension::Type::Unsigned64:
                {
                    ostr << getFieldInternal<uint64_t>(d, idx);
                    break;
                }
            case Dimension::Type::Float:
                {
                    ostr << getFieldInternal<float>(d, idx);
                    break;
                }
            case Dimension::Type::Double:
                {
                    ostr << getFieldInternal<double>(d, idx);
                    break;
                }
            default:
                throw;
            }
            ostr << endl;
        }
    }
}


std::ostream& operator<<(std::ostream& ostr, const PointBuffer& buf)
{
    buf.dump(ostr);
    return ostr;
}

} // namespace pdal
