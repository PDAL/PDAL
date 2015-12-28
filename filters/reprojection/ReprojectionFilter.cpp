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
#include <pdal/GlobalEnvironment.hpp>

#include <gdal.h>
#include <ogr_spatialref.h>

#include <memory>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.reprojection",
    "Reproject data using GDAL from one coordinate system to another.",
    "http://pdal.io/stages/filters.reprojection.html" );

CREATE_STATIC_PLUGIN(1, 0, ReprojectionFilter, Filter, s_info)

std::string ReprojectionFilter::getName() const { return s_info.name; }

ReprojectionFilter::ReprojectionFilter() : m_inferInputSRS(true),
    m_in_ref_ptr(NULL), m_out_ref_ptr(NULL), m_transform_ptr(NULL)
{}

ReprojectionFilter::~ReprojectionFilter()
{
    if (m_transform_ptr)
        OCTDestroyCoordinateTransformation(m_transform_ptr);
    if (m_in_ref_ptr)
        OSRDestroySpatialReference(m_in_ref_ptr);
    if (m_out_ref_ptr)
        OSRDestroySpatialReference(m_out_ref_ptr);
}

void ReprojectionFilter::processOptions(const Options& options)
{
    try
    {
       m_outSRS = options.getValueOrThrow<pdal::SpatialReference>("out_srs");
    }
    catch (std::invalid_argument)
    {
        std::string srs = options.getValueOrDefault<std::string>("out_srs", "");
        std::ostringstream oss;
        oss << "Stage " << getName() << " has invalid spatial reference "
            "specification for 'out_srs' option: '" << srs << "'.";
        throw pdal_error(oss.str());
    }
    catch (Option::not_found)
    {
        std::ostringstream oss;
        oss << "Stage " << getName() << " missing required option 'out_srs'.";
        throw Option::not_found(oss.str());
    }

    if (options.hasOption("in_srs"))
    {
        try
        {
            m_inSRS = options.getValueOrThrow<pdal::SpatialReference>("in_srs");
            m_inferInputSRS = false;
        }
        catch (std::invalid_argument)
        {
            std::string srs =
                options.getValueOrDefault<std::string>("in_srs", "");
            std::ostringstream oss;
            oss << "Stage " << getName() << " has invalid spatial reference "
                "specification for 'in_srs' option: '" << srs << "'.";
            throw pdal_error(oss.str());
        }
    }
}


void ReprojectionFilter::initialize()
{
    GlobalEnvironment::get().initializeGDAL(log(), isDebug());

    m_out_ref_ptr = OSRNewSpatialReference(0);

    int result = OSRSetFromUserInput(m_out_ref_ptr,
        m_outSRS.getWKT(pdal::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE)
    {
        std::ostringstream oss;
        oss << getName() << ": Invalid output spatial reference '" <<
            m_outSRS.getWKT() << "'.  This is usually caused by a bad value "
            "for the 'out_srs' option.";
        throw pdal_error(oss.str());
    }
}


void ReprojectionFilter::ready(PointTableRef table)
{
    if (!table.supportsView())
        createTransform(table.anySpatialReference());
}


void ReprojectionFilter::createTransform(const SpatialReference& srsSRS)
{
    if (m_inferInputSRS)
    {
        m_inSRS = srsSRS;
        if (m_inSRS.empty())
        {
            std::ostringstream oss;
            oss << getName() << ": source data has no spatial reference and "
                "none is specified with the 'in_srs' option.";
            throw pdal_error(oss.str());
        }
    }

    if (m_in_ref_ptr)
        OSRDestroySpatialReference(m_in_ref_ptr);
    m_in_ref_ptr = OSRNewSpatialReference(0);

    int result =
        OSRSetFromUserInput(m_in_ref_ptr,
            m_inSRS.getWKT(pdal::SpatialReference::eCompoundOK).c_str());
    if (result != OGRERR_NONE)
    {
        std::ostringstream oss;
        oss << getName() << ": Invalid input spatial reference '" <<
            m_inSRS.getWKT() << "'.  This is usually caused by a bad " <<
            "value for the 'in_srs' option or an invalid spatial reference " <<
            "in the source file.";
        throw pdal_error(oss.str());
    }
    if (m_transform_ptr)
        OCTDestroyCoordinateTransformation(m_transform_ptr);
    m_transform_ptr = OCTNewCoordinateTransformation(m_in_ref_ptr,
        m_out_ref_ptr);
    if (!m_transform_ptr)
    {
        std::ostringstream oss;
        oss << getName() << ": Could not construct transformation.";
        throw pdal_error(oss.str());
    }
}


bool ReprojectionFilter::transform(double& x, double& y, double& z)
{
    // OCTTransform will throw via GDAL error handler
    // if there is an error. We don't expect the return value
    // unless the GDAL handler gets shut off for whatever reason. In
    // that case, we'll just throw.
    if (OCTTransform(m_transform_ptr, 1, &x, &y, &z))
    {
        return true;
    }
    else
    {
        std::ostringstream msg;
        msg << "Could not project point for ReprojectionTransform::" <<
            CPLGetLastErrorMsg();
        throw pdal_error(msg.str());
    }
}


PointViewSet ReprojectionFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;
    PointViewPtr outView = view->makeNew();

    createTransform(view->spatialReference());

    double x, y, z;

    PointRef point(*view, 0);
    for (PointId id = 0; id < view->size(); ++id)
    {
        point.setPointId(id);
        if (processOne(point))
            outView->appendPoint(*view, id);
    }

    viewSet.insert(outView);
    view->setSpatialReference(m_outSRS);
    outView->setSpatialReference(m_outSRS);

    return viewSet;
}


bool ReprojectionFilter::processOne(PointRef& point)
{
    double x(point.getFieldAs<double>(Dimension::Id::X));
    double y(point.getFieldAs<double>(Dimension::Id::Y));
    double z(point.getFieldAs<double>(Dimension::Id::Z));

    if (OCTTransform(m_transform_ptr, 1, &x, &y, &z))
    {
        point.setField(Dimension::Id::X, x);
        point.setField(Dimension::Id::Y, y);
        point.setField(Dimension::Id::Z, z);

        return true;
    }
    else
    {
        return false;
    }
}

} // namespace pdal
