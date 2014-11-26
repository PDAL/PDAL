/******************************************************************************
* Copyright (c) 2014,  Hobu Inc., hobu@hobu.co
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
*     * Neither the name of Hobu, Inc. nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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

#include <pdal/DbWriter.hpp>

namespace pdal
{

void DbWriter::ready(PointContextRef ctx)
{
    using namespace Dimension;

    m_dimTypes = ctx.dimTypes();

    auto cmp = [](const DimType& d1, const DimType& d2) -> bool
    {
        long id1 = d1.m_id;
        long id2 = d2.m_id;

        // Put X, Y and Z at the end of the list.
        if (id1 == Id::X || id1 == Id::Y || id1 == Id::Z)
            id1 += 1000000;
        if (id2 == Id::X || id2 == Id::Y || id2 == Id::Z)
            id2 += 1000000;
        return id1 < id2;
    };

    // Sorting messes up the offsets in the DimType objects.
    std::sort(m_dimTypes.begin(), m_dimTypes.end(), cmp);

    // Now store away the offset of X, Y and Z if they exist.
    m_xPackedOffset = -1;
    m_yPackedOffset = -1;
    m_zPackedOffset = -1;
    int offset = 0;
    for (auto di = m_dimTypes.begin(); di != m_dimTypes.end(); ++di)
    {
        if (di->m_id == Id::X)
            m_xPackedOffset = offset;
        else if (di->m_id == Id::Y)
            m_yPackedOffset = offset;
        else if (di->m_id == Id::Z)
            m_zPackedOffset = offset;
        offset += Dimension::size(di->m_type);
    }
    m_packedPointSize = offset;
}


/// Get a dimension type list for the storage schema.
/// \return  Storage dimension types.
DimTypeList DbWriter::dbDimTypes() const
{
    using namespace Dimension;

    if (!locationScaling())
        return m_dimTypes;

    DimTypeList dimTypes(m_dimTypes);
    for (auto di = dimTypes.begin(); di != dimTypes.end(); ++di)
        if (di->m_id == Id::X || di->m_id == Id::Y || di->m_id == Id::Z)
            di->m_type = Type::Signed32;
    return dimTypes;
}


/// Read a point's data packed into a buffer.
/// \param[in] idx  Index of point to read.
/// \param[in] buf  Buffer to fill with data.
/// \return  Number of bytes written to buffer.
size_t DbWriter::readPoint(const PointBuffer& pb, PointId idx,
   std::vector<char>& outbuf)
{
    pb.getPackedPoint(m_dimTypes, idx, outbuf.data());
    
    auto iconvert = [](const XForm& xform, const char *inpos, char *outpos)
    {
        double d;
        int i;

        memcpy(&d, inpos, sizeof(double));
        d = (d - xform.m_offset) / xform.m_scale;
        i = boost::numeric_cast<int32_t>(lround(d));
        memcpy(outpos, &i, sizeof(i));
    };

    if (locationScaling())
    {
        int outOffset;

        if (m_xPackedOffset >= 0)
            outOffset = m_xPackedOffset;
        else if (m_yPackedOffset >= 0)
            outOffset = m_yPackedOffset;
        else if (m_zPackedOffset >= 0)
            outOffset = m_zPackedOffset;
        else
            outOffset = m_packedPointSize;  //So we return the proper size.

        if (m_xPackedOffset >= 0)
        {
            iconvert(m_xXform, outbuf.data() + m_xPackedOffset,
                outbuf.data() + outOffset);
            outOffset += sizeof(int);
        }
        if (m_yPackedOffset >= 0)
        {
            iconvert(m_yXform, outbuf.data() + m_yPackedOffset,
                outbuf.data() + outOffset);
            outOffset += sizeof(int);
        }
        if (m_zPackedOffset >= 0)
        {
            iconvert(m_zXform, outbuf.data() + m_zPackedOffset,
                outbuf.data() + outOffset);
            outOffset += sizeof(int);
        }
        return outOffset;
    }
    else
        return m_packedPointSize;
}

/// Determine if X, Y and Z values should be written as Signed32 along with
/// a scale factor and offset instead of being written as Double.
///
/// \return  Whether X,Y and Z values should be scaled.
bool DbWriter::locationScaling() const
{
    return (m_xXform.nonstandard() || m_yXform.nonstandard() ||
        m_zXform.nonstandard());
}

} // namespace pdal
