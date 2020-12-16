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
    args.add("in_axis_ordering", "Axis ordering override for in_srs", m_inAxisOrderingArg, {} );
    args.add("out_axis_ordering", "Axis ordering override for out_srs", m_outAxisOrderingArg, {} );
    args.add("error_on_failure", "Throw an exception if we can't reproject any point",
        m_errorOnFailure);
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

void ReprojectionFilter::prepared(PointTableRef table)
{

    // convert string args to integers and throw if
    // we can't
    auto convert = [] (const std::vector<std::string>& in)
    {
        std::vector<int> output;
        for (auto& str: in)
        {
           try
           {
                output.push_back(std::stoi(str));
           } catch (std::invalid_argument&)
           {
                throw pdal_error("Unable to convert axis ordering to integer");
           }

        }
        return output;

    };

    // Check that the sorted vector is 1,2 or 1,2,3
    auto check = [this] (const std::vector<int>& in)
    {
        auto test = in;
        std::sort(test.begin(), test.end());
        if (test.size() > 3)
            throwError("Axis ordering vector is too long");
        if (test.at(0) != 1 && test.at(1) != 2)
            throwError("Axis ordering is invalid");
        if (test.size() > 2)
            if (test.at(2) != 3)
                throwError("Axis ordering for 3rd dimension is invalid");

    };

    if (m_inAxisOrderingArg.size())
    {
        m_inAxisOrdering = convert(m_inAxisOrderingArg);
        check(m_inAxisOrdering);
    }

    if (m_outAxisOrderingArg.size())
    {
        m_outAxisOrdering = convert(m_outAxisOrderingArg);
        check(m_outAxisOrdering);
    }

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


    // If either vector is empty, GDAL's default ordering is used.
    if (m_inAxisOrdering.size() || m_outAxisOrdering.size())
    {

        m_transform.reset(new SrsTransform(m_inSRS,
                                           m_inAxisOrdering,
                                           m_outSRS,
                                           m_outAxisOrdering));
    } else {
        m_transform.reset(new SrsTransform(m_inSRS, m_outSRS));
    }
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
    else if (m_errorOnFailure)
        throwError("Couldn't reproject point with X/Y/Z coordinates of (" +
            std::to_string(point.getFieldAs<double>(Dimension::Id::X)) + ", " +
            std::to_string(point.getFieldAs<double>(Dimension::Id::Y)) + ", " +
            std::to_string(point.getFieldAs<double>(Dimension::Id::Z)) + ").");
    return ok;
}

} // namespace pdal
