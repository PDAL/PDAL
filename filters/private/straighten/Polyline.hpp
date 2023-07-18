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

#pragma once

#include <memory>
#include <pdal/Geometry.hpp>
#include <pdal/Log.hpp>
#include <pdal/PointRef.hpp>
#include <pdal/SpatialReference.hpp>

namespace pdal
{

class KD2Index;
class RowPointTable;
class PointView;

namespace straighten
{

// TODO : add a kdTree to look for segment more easily (using a pdal KDIndex ?)
class Polyline : public Geometry
{
public:
    Polyline();
    virtual ~Polyline();

    Polyline(const std::string& wkt_or_json,
             SpatialReference ref = SpatialReference());
    Polyline(OGRGeometryH g);
    Polyline(OGRGeometryH g, const SpatialReference& srs);
    Polyline(const Polyline& poly);
    Polyline& operator=(const Polyline& src);

    // give the closest point in segment on the polyline
    double closestSegment(const PointRef& point, double& x, double& y,
                          double& z, double& m, double& azimuth,
                          double& offset);

    // returns the given polyline point for a given PK as X point dimension
    void interpolate(const PointRef& point, double& x, double& y, double& z,
                     double& m, double& azimuth, double& offset);

    virtual void modified() override;
    virtual void clear() override;

private:
    void init();

    std::unique_ptr<KD2Index> m_index;
    std::unique_ptr<RowPointTable> m_table;
    std::unique_ptr<PointView> m_view;
};
} // namespace straighten
} // namespace pdal