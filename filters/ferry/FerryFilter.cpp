/******************************************************************************
* Copyright (c) 2014, Howard Butler, howard@hobu.co
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

#include "FerryFilter.hpp"

#include <pdal/pdal_export.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.ferry",
    "Copy date from one dimension to another.",
    "http://pdal.io/stages/filters.ferry.html" );

CREATE_STATIC_PLUGIN(1, 0, FerryFilter, Filter, s_info)

std::string FerryFilter::getName() const { return s_info.name; }

Options FerryFilter::getDefaultOptions()
{
    Options options;

    pdal::Option red("dimension", "Red", "");
    pdal::Option b0("to","Red2", "");
    pdal::Option def("def","Red2", "");
    pdal::Options redO;
    redO.add(b0);
    redO.add(def);
    red.setOptions(redO);

    options.add(red);

    return options;
}


void FerryFilter::processOptions(const Options& options)
{
    std::vector<Option> dimensions = options.getOptions("dimension");
    for (auto i = dimensions.begin(); i != dimensions.end(); ++i)
    {
        std::string name = i->getValue<std::string>();
        boost::optional<Options const&> dimensionOptions = i->getOptions();
        if (!dimensionOptions)
        {
            std::ostringstream oss;
            oss << "No 'to' dimension given for dimension '" <<
                name << "'";
            throw pdal_error(oss.str());
        }
        std::string to_dim =
            dimensionOptions->getValueOrThrow<std::string>("to");
        if (boost::algorithm::iequals(name, to_dim))
        {
            std::ostringstream oss;
            oss << "The from and to dimension cannot be the same"
                << " name for dimension '" << name << "'";
            throw pdal_error(oss.str());
        }
        m_name_map.insert(std::make_pair(name, to_dim));
    }
}
void FerryFilter::addDimensions(PointLayoutPtr layout)
{
    for (const auto& dim_par : m_name_map)
    {
        layout->registerOrAssignDim(dim_par.second, Dimension::Type::Double);
    }
}

void FerryFilter::ready(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    for (const auto& dim_par : m_name_map)
    {
        Dimension::Id::Enum f = layout->findDim(dim_par.first);
        Dimension::Id::Enum t = layout->findDim(dim_par.second);
        m_dimensions_map.insert(std::make_pair(f,t));
    }
}


void FerryFilter::filter(PointView& view)
{
    for (PointId id = 0; id < view.size(); ++id)
    {
        for (const auto& dim_par : m_dimensions_map)
        {
            double v = view.getFieldAs<double>(dim_par.first, id);
            view.setField(dim_par.second, id, v);
        }
    }
}


} // namespace pdal
