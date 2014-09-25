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

#include <pdal/filters/Ferry.hpp>

namespace pdal
{
namespace filters
{



void Ferry::initialize()
{
}


Options Ferry::getDefaultOptions()
{
    Options options;

    pdal::Option red("dimension", "Red", "");
    pdal::Option b0("band",1, "");
    pdal::Option s0("scale", 1.0f, "scale factor for this dimension");
    pdal::Options redO;
    redO.add(b0);
    redO.add(s0);
    red.setOptions(redO);

    pdal::Option green("dimension", "Green", "");
    pdal::Option b1("band",2, "");
    pdal::Option s1("scale", 1.0f, "scale factor for this dimension");
    pdal::Options greenO;
    greenO.add(b1);
    greenO.add(s1);
    green.setOptions(greenO);

    pdal::Option blue("dimension", "Blue", "");
    pdal::Option b2("band",3, "");
    pdal::Option s2("scale", 1.0f, "scale factor for this dimension");
    pdal::Options blueO;
    blueO.add(b2);
    blueO.add(s2);
    blue.setOptions(blueO);

    pdal::Option reproject("reproject", false,
        "Reproject the input data into the same coordinate system as "
        "the raster?");

    options.add(red);
    options.add(green);
    options.add(blue);
    options.add(reproject);

    return options;
}


void Ferry::processOptions(const Options& options)
{
    std::vector<Option> dimensions = options.getOptions("dimension");
//
//     if (dimensions.size() == 0)
//     {
//         m_bands.emplace_back("Red", Dimension::Id::Red, 1, 1.0);
//         m_bands.emplace_back("Green", Dimension::Id::Green, 2, 1.0);
//         m_bands.emplace_back("Blue", Dimension::Id::Blue, 3, 1.0);
//         log()->get(LogLevel::Debug) << "No dimension mappings were given. "
//             "Using default mappings." << std::endl;
//     }
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
        m_dimensions_map.insert(std::make_pair(name, to_dim));

    }
}


void Ferry::ready(PointContext ctx)
{
}


void Ferry::filter(PointBuffer& buffer)
{
}

void Ferry::done(PointContext ctx)
{
}

} // namespace filters
} // namespace pdal

