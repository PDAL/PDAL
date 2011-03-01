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

#include "libpc/CropFilter.hpp"

namespace libpc
{


CropFilter::CropFilter(Stage& prevStage, Bounds<double> const& bounds)
    : Filter(prevStage),
      m_bounds(bounds)
{
    Header& header = getHeader();
    header.setBounds(bounds);

    return;
}


const std::string& CropFilter::getName() const
{
    static std::string name("Crop Filter");
    return name;
}


boost::uint32_t CropFilter::readPoints(PointData& data)
{
    m_prevStage.readPoints(data);

    boost::uint32_t numPoints = data.getNumPoints();

    const SchemaLayout& schemaLayout = data.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();

    int fieldX = schema.getDimensionIndex(Dimension::Field_X);
    int fieldY = schema.getDimensionIndex(Dimension::Field_Y);
    int fieldZ = schema.getDimensionIndex(Dimension::Field_Z);

    boost::uint32_t numValidPoints = 0;

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        if (data.isValid(pointIndex))
        {
            float x = data.getField<float>(pointIndex, fieldX);
            float y = data.getField<float>(pointIndex, fieldY);
            float z = data.getField<float>(pointIndex, fieldZ);
            Vector<double> point(x,y,z);
            if (!m_bounds.contains(point))
            {
                // remove this point, and update the lower bound for Z
                data.setValid(pointIndex, false);
            }
            else
            {
                data.setValid(pointIndex, true);
                ++numValidPoints;
            }
        }
    }

    return numValidPoints;
}

}
