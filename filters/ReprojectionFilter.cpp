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

#include "ReprojectionFilter.hpp"

#include <pdal/PointView.hpp>
#include <pdal/private/SrsTransform.hpp>
#include <pdal/util/ProgramArgs.hpp>

//#include <memory>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.reprojection",
    "Reproject data using GDAL from one coordinate system to another.",
    "http://pdal.io/stages/filters.reprojection.html"
};

CREATE_STATIC_STAGE(ReprojectionFilter, s_info)

std::string ReprojectionFilter::getName() const { return s_info.name; }

ReprojectionFilter::ReprojectionFilter() : m_inferInputSRS(true)
{}


ReprojectionFilter::~ReprojectionFilter()
{}


void ReprojectionFilter::addArgs(ProgramArgs& args)
{
    args.add("out_srs", "Output spatial reference", m_outSRS).setPositional();
    args.add("in_srs", "Input spatial reference", m_inSRS);
}


void ReprojectionFilter::initialize()
{
    m_inferInputSRS = m_inSRS.empty();
    setSpatialReference(m_outSRS);
}


void ReprojectionFilter::spatialReferenceChanged(const SpatialReference& srs)
{
    createTransform(srs);
}


void ReprojectionFilter::createTransform(const SpatialReference& srsSRS)
{
    if (m_inferInputSRS)
    {
        m_inSRS = srsSRS;
        if (m_inSRS.empty())
            throwError("source data has no spatial reference and "
                "none is specified with the 'in_srs' option.");
    }
    m_transform.reset(new SrsTransform(m_inSRS, m_outSRS));
}


PointViewSet ReprojectionFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;
    PointViewPtr outView = view->makeNew();

    createTransform(view->spatialReference());

    PointRef point(*view, 0);
    for (PointId id = 0; id < view->size(); ++id)
    {
        point.setPointId(id);
        if (processOne(point))
            outView->appendPoint(*view, id);
    }

    viewSet.insert(outView);
    return viewSet;
}


bool ReprojectionFilter::processOne(PointRef& point)
{
    double x(point.getFieldAs<double>(Dimension::Id::X));
    double y(point.getFieldAs<double>(Dimension::Id::Y));
    double z(point.getFieldAs<double>(Dimension::Id::Z));

    bool ok = m_transform->transform(x, y, z);
    if (ok)
    {
        point.setField(Dimension::Id::X, x);
        point.setField(Dimension::Id::Y, y);
        point.setField(Dimension::Id::Z, z);
    }
    return ok;
}

} // namespace pdal
