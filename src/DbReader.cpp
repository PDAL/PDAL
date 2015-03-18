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

void DbReader::loadSchema(PointLayoutPtr layout,
    const std::string& schemaString)
{
    XMLSchema schema(schemaString);
    loadSchema(layout, schema);
}

void DbReader::loadSchema(PointLayoutPtr layout, const XMLSchema& schema)
{
    m_dims = schema.xmlDims();

    // Override XYZ to doubles and use those going forward
    // we will apply any scaling set before handing it off
    // to PDAL.
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);

    m_packedPointSize = 0;
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        di->m_dimType.m_id =
            layout->registerOrAssignDim(di->m_name, di->m_dimType.m_type);
        m_packedPointSize += Dimension::size(di->m_dimType.m_type);
    }
}


DimTypeList DbReader::dbDimTypes() const
{
    DimTypeList dimTypes;

    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
        dimTypes.push_back(di->m_dimType);
    return dimTypes;
}


void DbReader::writeField(PointView& view, const char *pos, const DimType& dim,
    PointId idx)
{
    using namespace Dimension;

    if (dim.m_type == Type::Signed32 &&
        (dim.m_id == Id::X || dim.m_id == Id::Y || dim.m_id == Id::Z))
    {
        int32_t i;

        memcpy(&i, pos, sizeof(int32_t));
        double d = (i * dim.m_xform.m_scale) + dim.m_xform.m_offset;
        view.setField(dim.m_id, idx, d);
    }
    else
    {
        view.setField(dim.m_id, dim.m_type, idx, pos);
    }
}


/// Write a point's packed data into a buffer.
/// \param[in] view PointView to write to.
/// \param[in] idx  Index of point to write.
/// \param[in] buf  Pointer to packed DB point data.
void DbReader::writePoint(PointView& view, PointId idx, const char *buf)
{
    using namespace Dimension;

    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        DimType dimType = di->m_dimType;
        // If we read X, Y or Z as a signed 32, apply the transform and write
        // the transformed value (double).
        if (dimType.m_type == Type::Signed32 &&
            (dimType.m_id == Id::X || dimType.m_id == Id::Y ||
             dimType.m_id == Id::Z))
        {
            int32_t i;

            memcpy(&i, buf, sizeof(int32_t));
            double d = (i * dimType.m_xform.m_scale) + dimType.m_xform.m_offset;
            view.setField(dimType.m_id, idx, d);
        }
        else
        {
            view.setField(dimType.m_id, dimType.m_type, idx, buf);
        }
        buf += Dimension::size(dimType.m_type);
    }
}

} // namespace pdal
