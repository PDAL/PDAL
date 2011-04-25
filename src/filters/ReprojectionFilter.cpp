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

#include <libpc/filters/ReprojectionFilter.hpp>

#include <libpc/Dimension.hpp>
#include <libpc/Schema.hpp>
#include <libpc/exceptions.hpp>
#include <libpc/PointBuffer.hpp>
#include <libpc/filters/ReprojectionFilterIterator.hpp>

namespace libpc { namespace filters {

ReprojectionFilter::ReprojectionFilter(const Stage& prevStage)
    : Filter(prevStage)
{
    checkImpedance();

    return;
}


void ReprojectionFilter::checkImpedance()
{
    return;
}


const std::string& ReprojectionFilter::getDescription() const
{
    static std::string name("Reprojection Filter");
    return name;
}

const std::string& ReprojectionFilter::getName() const
{
    static std::string name("filters.reprojection");
    return name;
}

void ReprojectionFilter::processBuffer(PointBuffer& data) const
{
    const boost::uint32_t numPoints = data.getNumPoints();

    //const SchemaLayout& schemaLayout = data.getSchemaLayout();
    //const Schema& schema = schemaLayout.getSchema();

    //const int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    //const Dimension& zDim = schema.getDimension(indexZ);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        //const boost::int32_t zraw = data.getField<boost::int32_t>(pointIndex, indexZ);
        //const double z = zDim.applyScaling(zraw);

        //data.setField<boost::uint16_t>(pointIndex, indexB, blue);

        data.setNumPoints(pointIndex+1);
    }

    return;
}


libpc::SequentialIterator* ReprojectionFilter::createSequentialIterator() const
{
    return new ReprojectionFilterSequentialIterator(*this);
}

} } // namespaces
