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

#include "private/georeference/LocalCartesian.hpp"
#include "private/georeference/Trajectory.hpp"
#include "private/georeference/Utils.hpp"
#include <pdal/filters/TransformationFilter.hpp>

#include "GeoreferenceFilter.hpp"

#include <pdal/pdal_internal.hpp>

namespace pdal
{
static PluginInfo const s_info{
    "filters.georeference", "Georeferencing filter",
    "http://pdal.io/stages/filters.georeference.html"};

CREATE_STATIC_STAGE(GeoreferenceFilter, s_info)

using DimId = Dimension::Id;

struct GeoreferenceFilter::Config
{
public:
    Trajectory m_trajectory;
    Eigen::Affine3d m_scan2imu;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Config(const std::string& trajFile,
           const TransformationFilter::Transform& matrix)
        : m_trajectory(trajFile)
    {
        Eigen::Matrix4d m;
        m << matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5],
            matrix[6], matrix[7], matrix[8], matrix[9], matrix[10], matrix[11],
            matrix[12], matrix[13], matrix[14], matrix[15];
        m_scan2imu.matrix() = m;
    }
};

GeoreferenceFilter::GeoreferenceFilter()
    : Filter(), Streamable(), m_matrix(new TransformationFilter::Transform),
      m_config(nullptr), m_localCartesian(new LocalCartesian(0.0, 0.0, 0.0)),
      m_trajectory(""), m_scan2imu(""), m_timeOffset(0.0), m_reverse(false)
{
}
GeoreferenceFilter::~GeoreferenceFilter() {}

std::string GeoreferenceFilter::getName() const
{
    return s_info.name;
}

void GeoreferenceFilter::addArgs(ProgramArgs& args)
{
    args.add("trajectory", "Path to trajectory file", m_trajectory)
        .setPositional();
    args.add("scan2imu", "Transformation from scanner to imu", *m_matrix)
        .setPositional();
    args.add("reverse", "reverse georeferencing", m_reverse, m_reverse);
    args.add("time_offset",
             "time offset between trajectory and scanner timestamps",
             m_timeOffset, m_timeOffset);
}

void GeoreferenceFilter::initialize()
{
    m_config.reset(new Config(m_trajectory, *m_matrix));
}

void GeoreferenceFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(DimId::ScanAngleRank);
}

void GeoreferenceFilter::prepared(PointTableRef table) {}

bool GeoreferenceFilter::processOne(PointRef& point)
{
    TrajPoint barycenter;
    if (!m_config->m_trajectory.getTrajPoint(
            point.getFieldAs<double>(Dimension::Id::GpsTime) + m_timeOffset,
            barycenter))
        return false;

    const Eigen::Vector3d scanAngle(
        point.getFieldAs<double>(Dimension::Id::BeamDirectionX),
        point.getFieldAs<double>(Dimension::Id::BeamDirectionY),
        point.getFieldAs<double>(Dimension::Id::BeamDirectionZ));

    point.setField(Dimension::Id::ScanAngleRank,
                   Utils::rad2deg(std::atan2(scanAngle.y(), scanAngle.x())));

    const Eigen::Affine3d transform(
        Utils::getTransformation(0.0, 0.0, 0.0, barycenter.roll,
                                 barycenter.pitch,
                                 barycenter.azimuth - barycenter.wanderAngle) *
        m_config->m_scan2imu);

    m_localCartesian->reset(Utils::rad2deg(barycenter.y),
                            Utils::rad2deg(barycenter.x), barycenter.z);
    if (m_reverse)
    {
        m_localCartesian->forward(point);
        // taking into account that scan2imu is given in ned ?
        const Eigen::Vector3d scan(
            transform.inverse() *
            Eigen::Vector3d(point.getFieldAs<double>(DimId::Y),
                            point.getFieldAs<double>(DimId::X),
                            -point.getFieldAs<double>(DimId::Z)));
        point.setField(DimId::X, scan.x());
        point.setField(DimId::Y, scan.y());
        point.setField(DimId::Z, scan.z());
    }
    else
    {
        const Eigen::Vector3d ned(
            transform * Eigen::Vector3d(point.getFieldAs<double>(DimId::X),
                                        point.getFieldAs<double>(DimId::Y),
                                        point.getFieldAs<double>(DimId::Z)));
        // taking into account that scan2imu is given in ned ?
        point.setField(DimId::X, ned.y());
        point.setField(DimId::Y, ned.x());
        point.setField(DimId::Z, -ned.z());
        m_localCartesian->reverse(point);
    }
    return true;
}

void GeoreferenceFilter::filter(PointView& view)
{
    PointRef point(view, 0);
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
    view.invalidateProducts();
}

} // namespace pdal