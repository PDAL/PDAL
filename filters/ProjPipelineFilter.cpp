/******************************************************************************
* Copyright (c) 2019, Aurelien Vila (aurelien.vila@delair.aero)
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

#include "ProjPipelineFilter.hpp"

#include <pdal/PointView.hpp>
#include <pdal/private/SrsTransform.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <ogr_spatialref.h>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.projpipeline",
    "Transform coordinates using Proj pipeline string, WKT2 coordinate operations or URN definition",
    "http://pdal.io/stages/filters.projpipeline.html"
};

CREATE_STATIC_STAGE(ProjPipelineFilter, s_info)

std::string ProjPipelineFilter::getName() const { return s_info.name; }

ProjPipelineFilter::ProjPipelineFilter()
{}


ProjPipelineFilter::~ProjPipelineFilter()
{}


void ProjPipelineFilter::addArgs(ProgramArgs& args)
{
    args.add("out_srs", "Output spatial reference", m_outSRS);
    args.add("reverse_transfo", "Wether the coordinate operation should be evaluated in the reverse path",
             m_reverseTransfo, false);
    args.add("coord_op", "Coordinate operation (Proj pipeline or WKT2 string or urn definition)",
             m_coordOperation).setPositional();
}


void ProjPipelineFilter::initialize()
{
    setSpatialReference(m_outSRS);
    createTransform(m_coordOperation, m_reverseTransfo);
}


void ProjPipelineFilter::createTransform(const std::string coordOperation, bool reverseTransfo)
{
    m_coordTransform.reset(new CoordTransform(coordOperation, reverseTransfo));
}


PointViewSet ProjPipelineFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;
    PointViewPtr outView = view->makeNew();

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


bool ProjPipelineFilter::processOne(PointRef& point)
{
    double x(point.getFieldAs<double>(Dimension::Id::X));
    double y(point.getFieldAs<double>(Dimension::Id::Y));
    double z(point.getFieldAs<double>(Dimension::Id::Z));

    bool ok = m_coordTransform->transform(x, y, z);
    if (ok)
    {
        point.setField(Dimension::Id::X, x);
        point.setField(Dimension::Id::Y, y);
        point.setField(Dimension::Id::Z, z);
    }
    return ok;
}

ProjPipelineFilter::CoordTransform::CoordTransform(){}

ProjPipelineFilter::CoordTransform::CoordTransform(const std::string coordOperation, bool reverseTransfo){
    OGRCoordinateTransformationOptions coordTransfoOptions;
    coordTransfoOptions.SetCoordinateOperation(coordOperation.c_str(), reverseTransfo);
    OGRSpatialReference nullSrs("");
    m_transform.reset(OGRCreateCoordinateTransformation(&nullSrs, &nullSrs, coordTransfoOptions));
}

bool ProjPipelineFilter::CoordTransform::transform(double &x, double &y, double &z){

    return m_transform && m_transform->Transform(1, &x, &y, &z);
}

} // namespace pdal

