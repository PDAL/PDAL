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

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <pdal/filters/Scaling.hpp>

namespace
{

pdal::dimension::Interpretation stringToType(std::string t)
{
    using namespace pdal;

    std::transform(t.begin(), t.end(), t.begin(), ::tolower);
    if (t == "signedinteger")
        return dimension::SignedInteger;
    if (t == "unsignedinteger")
        return dimension::UnsignedInteger;
    if (t == "float")
        return dimension::Float;
    return dimension::Undefined;
}

} //namespace


namespace pdal
{
namespace filters
{

void Scaling::processOptions(const Options& options)
{
    std::vector<Option> dimensions = options.getOptions("dimension");

    for (size_t i = 0; i < dimensions.size(); ++i)
    {
        Scaler scaler;
        Option& dim = dimensions[i];
        boost::optional<Options const&> dimOps = dim.getOptions();
        scaler.name = dim.getValue<std::string>();
        if (dimOps)
        {
            scaler.scale = dimOps->getValueOrDefault<std::string>("scale", "");
            scaler.offset =
                dimOps->getValueOrDefault<std::string>("offset", "");
            scaler.type =
                dimOps->getValueOrDefault<std::string>("type", "SignedInteger");
            scaler.size = dimOps->getValueOrDefault<uint32_t>("size", 4);
        }
        m_scalers.push_back(scaler);
    }
    m_markIgnored = options.getValueOrDefault<bool>(
        "ignore_old_dimensions", true);
}


void Scaling::buildSchema(Schema *schema)
{
    // Loop through the options that the filter.Scaler collected. For each
    // dimension described, create a new dimension with the given parameters.
    // Create a map with the uuid of the new dimension that maps to the old
    // dimension to be scaled

    for (auto si = m_scalers.begin(); si != m_scalers.end(); ++si)
    {
        Scaler& s = *si;
        DimensionPtr fromDim = schema->getDimension(s.name);
        if (!fromDim)
            continue;
        Dimension dim(*fromDim);
        dim.setInterpretation(stringToType(s.type));
        dim.setByteSize(s.size);
        if (s.scale.size())
            dim.setNumericScale(boost::lexical_cast<double>(s.scale));
        if (s.offset.size())
            dim.setNumericOffset(boost::lexical_cast<double>(s.offset));
        dim.createUUID();
        dim.setNamespace(getName());
        schema->appendDimension(dim);
        if (m_markIgnored)
            fromDim->setIgnored();
        m_dims.push_back(DimInfo(fromDim->getName(), fromDim->getNamespace()));
    }
}


void Scaling::ready(PointContext ctx)
{
    Schema *s = ctx.schema();

    // Fill info with dim pointers.
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        DimInfo& dimInfo = *di;
        dimInfo.from = s->getDimension(dimInfo.name, dimInfo.fromNs);
        dimInfo.to = s->getDimension(dimInfo.name, getName());
    }
}


void Scaling::filter(PointBuffer& buf)
{
    for (PointId idx = 0; idx < buf.size(); ++idx)
    {
        for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
        {
            DimInfo& dimInfo = *di;
            double d = buf.getFieldAs<double>(dimInfo.from, idx);
            buf.setFieldUnscaled(dimInfo.to, idx, d);
        }
    }
}


} // namespace filters
} // namespace pdal
