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

#include <pdal/DbReader.hpp>

namespace pdal
{

void DbReader::loadSchema(PointContextRef ctx, const std::string& schemaString)
{
    XMLSchema schema;
    schema.read(schemaString);

    m_dims = schema.dims();
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
        di->m_id = ctx.registerOrAssignDim(di->m_name, di->m_type);
}


/// Write a point's packed data into a buffer.
/// \param[in] pb  PointBuffer to write to.
/// \param[in] idx  Index of point to write.
/// \param[in] buf  Pointer to packed DB point data.
void DbReader::writePoint(PointBuffer& pb, PointId idx, const char *buf)
{
    using namespace Dimension;

    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        // If we read X, Y or Z as a signed 32, apply the transform and write
        // the transformed value (double).
        if (di->m_type == Type::Signed32 &&
            (di->m_id == Id::X || di->m_id == Id::Y || di->m_id == Id::Z))
        {
            int32_t i;

            memcpy(&i, buf, sizeof(int32_t));
            double d = (i * di->m_xform.m_scale) + di->m_xform.m_offset;
            pb.setField(di->m_id, idx, d);
        }
        else
            pb.setField(di->m_id, di->m_type, idx, buf);
        buf += Dimension::size(di->m_type);
    }
}

} // namespace pdal
