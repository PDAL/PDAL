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

#include "CropFilter.hpp"

#include <iomanip>

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Polygon.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <sstream>
#include <cstdarg>
#include <thread>
#include <functional>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.crop",
    "Filter points inside or outside a bounding box or a polygon if PDAL was built with GEOS support.",
    "http://pdal.io/stages/filters.crop.html" );

std::vector<unsigned int> bounds(unsigned int parts, unsigned int mem) {
    std::vector<unsigned int>bnd;
    unsigned int delta = mem / parts;
    unsigned int reminder = mem % parts;
    unsigned int N1 = 0, N2 = 0;
    bnd.push_back(N1);
    for (unsigned int i = 0; i < parts; ++i) {
        N2 = N1 + delta;
        if (i == parts - 1)
            N2 += reminder;
        bnd.push_back(N2);
        N1 = N2;
    }
    return bnd;
}

CREATE_STATIC_PLUGIN(1, 0, CropFilter, Filter, s_info)

std::string CropFilter::getName() const { return s_info.name; }

CropFilter::CropFilter() : pdal::Filter()
{
    m_cropOutside = false;
}

void CropFilter::addArgs(ProgramArgs& args)
{
    args.add("outside", "Whether we keep points inside or outside of the "
        "bounding region", m_cropOutside);
    args.add("a_srs", "Spatial reference for bounding region", m_assignedSrs);
    args.add("bounds", "Bounds box for cropped points", m_bounds);
    args.add("polygon", "Bounding polying for cropped points", m_polys).
        setErrorText("Invalid polygon specification.  "
            "Must be valid GeoJSON/WKT");
    args.add("num_threads", "Number of threads to use for the crop operation", m_numthreads, 1);
}


void CropFilter::initialize()
{
    // Set geometry from polygons.
    if (m_polys.size())
    {
        m_geoms.clear();
        for (Polygon& poly : m_polys)
        {
            GeomPkg g;

            // Throws if invalid.
            poly.valid();
            if (!m_assignedSrs.empty())
                poly.setSpatialReference(m_assignedSrs);
            g.m_geom = poly;
            m_geoms.push_back(g);
        }
    }
}


void CropFilter::ready(PointTableRef table)
{
    for (auto& geom : m_geoms)
    {
        // If we already overrode the SRS, use that instead
        if (m_assignedSrs.empty())
            geom.m_geom.setSpatialReference(table.anySpatialReference());
    }
}


bool CropFilter::processOne(PointRef& point)
{
    for (auto& geom : m_geoms)
        if (!crop(point, geom))
            return false;

    for (auto& box : m_bounds)
        if (!crop(point, box.to2d()))
            return false;

    return true;
}


PointViewSet CropFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;
    SpatialReference srs = view->spatialReference();

    // Don't do anything if no bounds have been specified.
    if (m_geoms.empty() && m_bounds.empty())
    {
        viewSet.insert(view);
        return viewSet;
    }

    for (auto& geom : m_geoms)
    {
        // If this is the first time through or the SRS has changed,
        // prepare the crop polygon.
        if (srs != m_lastSrs)
        {
            geom.m_geom = geom.m_geom.transform(srs);
        }

        PointViewPtr outView = view->makeNew();
        crop(geom, *view, *outView);
        viewSet.insert(outView);
    }
    m_lastSrs = srs;

    for (auto& box : m_bounds)
    {
        PointViewPtr outView = view->makeNew();
        crop(box.to2d(), *view, *outView);
        viewSet.insert(outView);
    }
    return viewSet;
}


bool CropFilter::crop(PointRef& point, const BOX2D& box)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);

    // Return true if we're keeping a point.
    return (m_cropOutside != box.contains(x, y));
}

void CropFilter::cropJobBox(unsigned int left, unsigned int right, const BOX2D& box, PointView& input, std::vector<bool>& results) {
  PointRef point = input.point(0);
  for (PointId idx = left; idx < right; ++idx){
    point.setPointId(idx);
    results[static_cast<int>(idx)] = crop(point, box);
  }
}

void CropFilter::crop(const BOX2D& box, PointView& input, PointView& output)
{
    if (m_numthreads == 1){
      PointRef point = input.point(0);
      for (PointId idx = 0; idx < input.size(); ++idx)
      {
          point.setPointId(idx);
          if (crop(point, box))
              output.appendPoint(input, idx);
      }
    }else{
      std::vector<unsigned int>bnd = bounds(m_numthreads, input.size());

      std::vector<std::thread> tt;

      std::vector<bool> result;
      result.reserve(input.size());

      for (int i = 0; i < m_numthreads - 1; ++i)
        tt.push_back(std::thread(&CropFilter::cropJobBox, this, bnd[i], bnd[i + 1], std::ref(box), std::ref(input), std::ref(result)));

      for (int i = m_numthreads - 1; i < m_numthreads; ++i)
         cropJobBox(bnd[i], bnd[i + 1], box, input, result);

      for(auto &e : tt) e.join();

      for (PointId idx = 0; idx < input.size(); ++idx){
        if (result[static_cast<int>(idx)] == true)
          output.appendPoint(input, idx);
      }
    }
}

bool CropFilter::crop(PointRef& point, const GeomPkg& g)
{
    bool covers = g.m_geom.covers(point);
    bool keep = (m_cropOutside != covers);
    return keep;
}

void CropFilter::cropJobGeom(unsigned int left, unsigned int right, const GeomPkg& g, PointView& input, std::vector<bool>& results){

    PointRef point = input.point(0);
    for (PointId idx = left; idx < right; ++idx){
        point.setPointId(idx);
        // bool covers = g.m_geom.covers(point);
        // results[static_cast<int>(idx)] = (m_cropOutside != covers);
        results[static_cast<int>(idx)] = crop(point, g);
    }
}


void CropFilter::crop(const GeomPkg& g, PointView& input, PointView& output)
{
    if (m_numthreads == 1){
      PointRef point = input.point(0);
      for (PointId idx = 0; idx < input.size(); ++idx){
          point.setPointId(idx);
          bool covers = g.m_geom.covers(point);
          bool keep = (m_cropOutside != covers);
          if (keep)
              output.appendPoint(input, idx);
      }
    }
    else{

      std::vector<unsigned int>bnd = bounds(m_numthreads, input.size());

      std::vector<std::thread> tt;

      std::vector<bool> result;
      result.reserve(input.size());

      CropFilter* cropFilter = this;

      for (int i = 0; i < m_numthreads - 1; ++i)
        tt.push_back(std::thread(&CropFilter::cropJobGeom, this, bnd[i], bnd[i + 1], std::ref(g), std::ref(input), std::ref(result)));

      for (int i = m_numthreads - 1; i < m_numthreads; ++i)
         cropJobGeom(bnd[i], bnd[i + 1], g, input, result);

      for(auto &e : tt) e.join();

      for (PointId idx = 0; idx < input.size(); ++idx){
        if (result[static_cast<int>(idx)] == true)
          output.appendPoint(input, idx);
      }
    }
}



} // namespace pdal
