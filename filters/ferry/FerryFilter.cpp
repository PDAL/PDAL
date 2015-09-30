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

    options.add("dimensions", "", "Dimensions to copy (<in>=<out>,...)");

    return options;
}


void FerryFilter::processOptions(const Options& options)
{
    if (options.hasOption("dimension"))
        throw pdal_error("Option 'dimension' no longer supported.  Use "
            "'dimensions' instead.");

    StringList dims = options.getValueOrThrow<StringList>("dimensions");
    for (auto& dim : dims)
    {
        StringList s = Utils::split2(dim, '=');
        if (s.size() != 2)
        {
            std::ostringstream oss;
            oss << "Invalid dimension specified '" << dim <<
                "'.  Need <from dimension>=<to dimension>.  See "
                "documentation for details.";
            throw pdal_error(oss.str());
        }
        Utils::trim(s[0]);
        Utils::trim(s[1]);
        if (s[0] == s[1])
        {
            std::ostringstream oss;
            oss << "Can't ferry dimension '" << s[0] << "' to itself.";
            throw pdal_error(oss.str());
        }
        m_name_map[s[0]] = s[1];
    }
}


void FerryFilter::addDimensions(PointLayoutPtr layout)
{
    for (const auto& dim_par : m_name_map)
    {
        layout->registerOrAssignDim(dim_par.second, Dimension::Type::Double);
    }
}


void FerryFilter::prepared(PointTableRef table)
{
    for (const auto& dims : m_name_map)
        if (table.layout()->findDim(dims.first) == Dimension::Id::Unknown)
        {
            std::ostringstream oss;
            oss << "Can't ferry dimension '" << dims.first << "'. "
                "Dimension doesn't exist.";
            throw pdal_error(oss.str());
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

