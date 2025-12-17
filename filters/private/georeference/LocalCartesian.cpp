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

namespace pdal
{
namespace georeference
{

LocalCartesian::LocalCartesian(const double lat0, const double lon0,
                               const double h0)
    : m_ctx(proj_context_create()), m_ecef2enu(nullptr)
{
    std::string deg2rad("+proj=unitconvert +xy_in=deg +xy_out=rad");
    m_deg2rad = proj_create(m_ctx, deg2rad.c_str());
    std::string source2ecef("+proj=cart +ellps=WGS84");
    m_source2ecef = proj_create(m_ctx, source2ecef.c_str());
    reset(lat0, lon0, h0);
}

LocalCartesian::~LocalCartesian()
{

    proj_destroy(m_deg2rad);
    proj_destroy(m_source2ecef);
    proj_destroy(m_ecef2enu);

    proj_context_destroy(m_ctx);
}

void LocalCartesian::reset(const double lat0, const double lon0,
                           const double h0)
{
    if (m_ecef2enu)
        proj_destroy(m_ecef2enu);
    std::stringstream ss;
    ss << std::fixed << std::setprecision(12)
       << "+proj=topocentric +ellps=WGS84"
       << " +lon_0=" << lon0 << " +lat_0=" << lat0 << " +h_0=" << h0;
    m_ecef2enu = proj_create(m_ctx, ss.str().c_str());
}

void LocalCartesian::forward(double& x, double& y, double& z) const
{
    PJ_COORD c = {{x, y, z, HUGE_VAL}};
    c = proj_trans(
        m_ecef2enu, PJ_FWD,
        proj_trans(m_source2ecef, PJ_FWD, proj_trans(m_deg2rad, PJ_FWD, c)));
    x = c.v[0];
    y = c.v[1];
    z = c.v[2];
}

void LocalCartesian::reverse(double& x, double& y, double& z) const
{
    PJ_COORD c = {{x, y, z, HUGE_VAL}};
    c = proj_trans(
        m_deg2rad, PJ_INV,
        proj_trans(m_source2ecef, PJ_INV, proj_trans(m_ecef2enu, PJ_INV, c)));
    x = c.v[0];
    y = c.v[1];
    z = c.v[2];
}

void LocalCartesian::forward(PointRef& point) const
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);
    forward(x, y, z);
    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
}

void LocalCartesian::reverse(PointRef& point) const
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);
    reverse(x, y, z);
    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
}

} // namespace georeference
} // namespace pdal
