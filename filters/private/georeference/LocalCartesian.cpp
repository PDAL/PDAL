/******************************************************************************
 * Copyright (c) 2023, Guilhem Villemin (guilhem.villemin@altametris.com)
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
 *     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include "LocalCartesian.hpp"
#include <ogr_spatialref.h>
#include <ostream>
#include <proj/io.hpp>

namespace pdal
{
namespace georeference
{

std::string enuProj(const double lat0, const double lon0, const double h0 = 0.0)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(12)
       << "+proj=topocentric +ellps=WGS84"
       << " +lon_0=" << lon0 << " +lat_0=" << lat0 << " +h_0=" << h0;
    return ss.str();
}

// NNPtr has no default ctor so we must init all that here
LocalCartesian::LocalCartesian(const double lat0, const double lon0,
                               const double h0)
    : m_ctx(proj_context_create()),
      m_source2ecef(NN_CHECK_THROW(nn_dynamic_pointer_cast<CoordinateOperation>(
                                       createFromUserInput(
                                           "+proj=cart +ellps=WGS84", m_ctx)))
                        ->coordinateTransformer(m_ctx)),
      m_ecef2source(
          NN_CHECK_THROW(
              nn_dynamic_pointer_cast<CoordinateOperation>(
                  createFromUserInput("+proj=cart +ellps=WGS84 +inv", m_ctx)))
              ->coordinateTransformer(m_ctx)),
      m_deg2rad(
          NN_CHECK_THROW(
              nn_dynamic_pointer_cast<CoordinateOperation>(createFromUserInput(
                  "+proj=unitconvert +xy_in=deg +xy_out=rad", m_ctx)))
              ->coordinateTransformer(m_ctx)),
      m_rad2deg(
          NN_CHECK_THROW(
              nn_dynamic_pointer_cast<CoordinateOperation>(createFromUserInput(
                  "+proj=unitconvert +xy_in=rad +xy_out=deg", m_ctx)))
              ->coordinateTransformer(m_ctx)),
      m_ecef2enu(NN_CHECK_THROW(
                     nn_dynamic_pointer_cast<CoordinateOperation>(
                         createFromUserInput(enuProj(lat0, lon0, h0), m_ctx)))
                     ->coordinateTransformer(m_ctx)),
      m_enu2ecef(NN_CHECK_THROW(
                     nn_dynamic_pointer_cast<CoordinateOperation>(
                         createFromUserInput(enuProj(lat0, lon0, h0), m_ctx)))
                     ->inverse()
                     ->coordinateTransformer(m_ctx))

{
}

LocalCartesian::~LocalCartesian()
{
    proj_context_destroy(m_ctx);
}

void LocalCartesian::reset(const double lat0, const double lon0,
                           const double h0)
{
    auto ecef2enuOperation =
        NN_CHECK_THROW(nn_dynamic_pointer_cast<CoordinateOperation>(
            createFromUserInput(enuProj(lat0, lon0, h0), m_ctx)));
    m_ecef2enu = ecef2enuOperation->coordinateTransformer(m_ctx);
    m_enu2ecef = ecef2enuOperation->inverse()->coordinateTransformer(m_ctx);
};

void LocalCartesian::forward(PointRef& point)
{
    PJ_COORD c = {{point.getFieldAs<double>(Dimension::Id::X),
                   point.getFieldAs<double>(Dimension::Id::Y),
                   point.getFieldAs<double>(Dimension::Id::Z), HUGE_VAL}};

    std::cout << __PRETTY_FUNCTION__ << " " << c.v[0] << " " << c.v[1] << " "
              << c.v[2] << std::endl;
    c = m_ecef2enu->transform(
        m_source2ecef->transform(m_deg2rad->transform(c)));
    std::cout << __PRETTY_FUNCTION__ << " " << c.v[0] << " " << c.v[1] << " "
              << c.v[2] << std::endl;
    point.setField(Dimension::Id::X, c.v[0]);
    point.setField(Dimension::Id::Y, c.v[1]);
    point.setField(Dimension::Id::Z, c.v[2]);
};

void LocalCartesian::reverse(PointRef& point)
{
    PJ_COORD c = {{point.getFieldAs<double>(Dimension::Id::X),
                   point.getFieldAs<double>(Dimension::Id::Y),
                   point.getFieldAs<double>(Dimension::Id::Z), HUGE_VAL}};

    c = m_rad2deg->transform(
        m_ecef2source->transform(m_enu2ecef->transform(c)));

    point.setField(Dimension::Id::X, c.v[0]);
    point.setField(Dimension::Id::Y, c.v[1]);
    point.setField(Dimension::Id::Z, c.v[2]);
}

} // namespace georeference
} // namespace pdal