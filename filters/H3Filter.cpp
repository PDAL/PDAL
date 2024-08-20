/******************************************************************************
 * Copyright (c) 2024, Howard Butler (info@hobu.co)
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

#include "H3Filter.hpp"

#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/Utils.hpp>

#include <pdal/private/SrsTransform.hpp>

#include <cctype>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <h3api.h>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.h3",
    "Compute H3 indexes for points",
    "http://pdal.io/stages/filters.h3.html"
};

CREATE_STATIC_STAGE(H3Filter, s_info)

std::string H3Filter::getName() const
{
    return s_info.name;
}

struct H3Filter::Args
{
    int m_resolution;
};


H3Filter::H3Filter() : m_args(new Args)
{}


H3Filter::~H3Filter()
{}


void H3Filter::addArgs(ProgramArgs& args)
{
    // Set resolution and such
    args.add("resolution",
        "H3 resolution parameter",
        m_args->m_resolution).setPositional();
}



void H3Filter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::H3);
}

bool H3Filter::processOne(PointRef& point)
{



    double x(point.getFieldAs<double>(Dimension::Id::X));
    double y(point.getFieldAs<double>(Dimension::Id::Y));
    double z(0.0);

    LatLng ll;
    H3Index index(0);
    bool ok = m_transform->transform(x, y, z);
    if (ok)
    {
        // x is longitude
        double xrad = PDALH3degsToRads(x);
        double yrad = PDALH3degsToRads(y);
        ll.lat = yrad;
        ll.lng = xrad;

        H3Error err = PDALH3latLngToCell(&ll, m_args->m_resolution, &index);
        if (err == E_SUCCESS)
        {
            point.setField(Dimension::Id::H3, index);
        } else
        {
            throwError("Unable to compute H3 cell id for point!");
        }
    }
    else
        throwError("Couldn't reproject point with X/Y/Z coordinates of (" +
            std::to_string(point.getFieldAs<double>(Dimension::Id::X)) + ", " +
            std::to_string(point.getFieldAs<double>(Dimension::Id::Y)) + ").");
    return ok;
}

void H3Filter::spatialReferenceChanged(const SpatialReference& srs)
{
    createTransform(srs);
}

void H3Filter::createTransform(const SpatialReference& srsSRS)
{
    if (srsSRS.empty())
        throwError("source data has no spatial reference");

    m_transform.reset(new SrsTransform(srsSRS, "EPSG:4326"));
}

void H3Filter::filter(PointView& view)
{
    if (!m_transform)
    {
        createTransform(view.spatialReference());
    }

    PointRef point = view.point(0);
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
}


} // namespace pdal
