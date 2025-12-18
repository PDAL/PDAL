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

#include "TransformationFilter.hpp"
#include "private/georeference/LocalCartesian.hpp"
#include "private/georeference/Trajectory.hpp"
#include "private/georeference/Utils.hpp"
#include <nlohmann/json.hpp>

#include "GeoreferenceFilter.hpp"

namespace pdal
{
static PluginInfo const s_info{
    "filters.georeference", "Georeferencing filter",
    "https://pdal.org/stages/filters.georeference.html"};

CREATE_STATIC_STAGE(GeoreferenceFilter, s_info)

using DimId = Dimension::Id;

struct GeoreferenceFilter::Config
{
public:
    TransformationFilter::Transform m_matrix;
    std::string m_trajectoryFile;
    NL::json m_trajectoryOptions;
    std::string m_coordinateSystem;
    double m_timeOffset;
    bool m_reverse;
    bool m_transformBeam;
    std::unique_ptr<georeference::Trajectory> m_trajectory;
    Eigen::Affine3d m_scan2imu;
    bool m_ned;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Config()
        : m_trajectoryFile(""), m_coordinateSystem("NED"), m_timeOffset(0.0),
          m_reverse(false), m_transformBeam(false), m_trajectory(nullptr)
    {
    }
    void init()
    {
        m_trajectory.reset(new georeference::Trajectory(m_trajectoryFile,
                                                        m_trajectoryOptions));
        Eigen::Matrix4d m;
        m << m_matrix[0], m_matrix[1], m_matrix[2], m_matrix[3], m_matrix[4],
            m_matrix[5], m_matrix[6], m_matrix[7], m_matrix[8], m_matrix[9],
            m_matrix[10], m_matrix[11], m_matrix[12], m_matrix[13],
            m_matrix[14], m_matrix[15];
        m_scan2imu.matrix() = m;
        std::string s = Utils::toupper(m_coordinateSystem);
        if (s == "NED")
            m_ned = true;
        else if (s == "ENU")
            m_ned = false;
        else
            throw pdal_error("Local Tangent Plane coordinate system " +
                             m_coordinateSystem + " is not allowed.");
    }
};

GeoreferenceFilter::GeoreferenceFilter()
    : Filter(), Streamable(), m_config(new Config),
      m_localCartesian(new georeference::LocalCartesian(0.0, 0.0, 0.0))

{
}
GeoreferenceFilter::~GeoreferenceFilter() {}

std::string GeoreferenceFilter::getName() const
{
    return s_info.name;
}

void GeoreferenceFilter::addArgs(ProgramArgs& args)
{
    args.add("trajectory_file", "Path to trajectory file",
             m_config->m_trajectoryFile)
        .setPositional();
    args.add("trajectory_options", "Trajectory reader option",
             m_config->m_trajectoryOptions);
    args.add("scan2imu", "Transformation from scanner to imu",
             m_config->m_matrix)
        .setPositional();
    args.add("reverse", "reverse georeferencing", m_config->m_reverse,
             m_config->m_reverse);
    args.add("time_offset",
             "time offset between trajectory and scanner timestamps",
             m_config->m_timeOffset, m_config->m_timeOffset);
    args.add("coordinate_system", "scan2imu coordinate system",
             m_config->m_coordinateSystem, m_config->m_coordinateSystem);
    args.add("transform_beam",
             "Transform BeamOrigin and BeamDirection dimensions",
             m_config->m_transformBeam, m_config->m_transformBeam);
}

void GeoreferenceFilter::initialize()
{
    m_config->init();
}

void GeoreferenceFilter::prepared(PointTableRef table)
{
    if (m_config->m_transformBeam)
    {
        // Verify all beam dimensions are present
        if (!table.layout()->hasDim(DimId::BeamOriginX) ||
            !table.layout()->hasDim(DimId::BeamOriginY) ||
            !table.layout()->hasDim(DimId::BeamOriginZ) ||
            !table.layout()->hasDim(DimId::BeamDirectionX) ||
            !table.layout()->hasDim(DimId::BeamDirectionY) ||
            !table.layout()->hasDim(DimId::BeamDirectionZ))
        {
            throwError("transform_beam option requires BeamOriginX/Y/Z and "
                       "BeamDirectionX/Y/Z dimensions to be present in the "
                       "point data.");
        }
    }
}

bool GeoreferenceFilter::processOne(PointRef& point)
{
    georeference::TrajPoint barycenter;
    if (!m_config->m_trajectory->getTrajPoint(
            point.getFieldAs<double>(Dimension::Id::GpsTime) +
                m_config->m_timeOffset,
            barycenter))
        return false;

    const Eigen::Affine3d transform(
        georeference::Utils::getTransformation(
            0.0, 0.0, 0.0, barycenter.roll, barycenter.pitch,
            barycenter.azimuth - barycenter.wanderAngle) *
        m_config->m_scan2imu);

    m_localCartesian->reset(georeference::Utils::rad2deg(barycenter.y),
                            georeference::Utils::rad2deg(barycenter.x),
                            barycenter.z);
    if (m_config->m_reverse)
    {
        m_localCartesian->forward(point);
        const Eigen::Vector3d scan(
            transform.inverse() *
            (m_config->m_ned
                 ? Eigen::Vector3d(point.getFieldAs<double>(DimId::Y),
                                   point.getFieldAs<double>(DimId::X),
                                   -point.getFieldAs<double>(DimId::Z))
                 : Eigen::Vector3d(point.getFieldAs<double>(DimId::X),
                                   point.getFieldAs<double>(DimId::Y),
                                   point.getFieldAs<double>(DimId::Z))));
        point.setField(DimId::X, scan.x());
        point.setField(DimId::Y, scan.y());
        point.setField(DimId::Z, scan.z());

        // Transform BeamOrigin (point) - same as X/Y/Z
        if (m_config->m_transformBeam)
        {
            // Create temp point for BeamOrigin transformation
            double origX = point.getFieldAs<double>(DimId::BeamOriginX);
            double origY = point.getFieldAs<double>(DimId::BeamOriginY);
            double origZ = point.getFieldAs<double>(DimId::BeamOriginZ);
            double dirX =
                origX + point.getFieldAs<double>(DimId::BeamDirectionX);
            double dirY =
                origY + point.getFieldAs<double>(DimId::BeamDirectionY);
            double dirZ =
                origZ + point.getFieldAs<double>(DimId::BeamDirectionZ);
            m_localCartesian->forward(origX, origY, origZ);
            m_localCartesian->forward(dirX, dirY, dirZ);

            // Apply same transformation as for X/Y/Z
            const Eigen::Vector3d scanOrig(
                transform.inverse() *
                (m_config->m_ned ? Eigen::Vector3d(origY, origX, -origZ)
                                 : Eigen::Vector3d(origX, origY, origZ)));

            const Eigen::Vector3d scanDir(
                transform.inverse() *
                (m_config->m_ned ? Eigen::Vector3d(dirY, dirX, -dirZ)
                                 : Eigen::Vector3d(dirX, dirY, dirZ)));
            point.setField(DimId::BeamOriginX, scanOrig.x());
            point.setField(DimId::BeamOriginY, scanOrig.y());
            point.setField(DimId::BeamOriginZ, scanOrig.z());
            const Eigen::Vector3d direction = (scanDir - scanOrig).normalized();
            point.setField(DimId::BeamDirectionX, direction.x());
            point.setField(DimId::BeamDirectionY, direction.y());
            point.setField(DimId::BeamDirectionZ, direction.z());
        }
    }
    else
    {
        const Eigen::Vector3d ned(
            transform * Eigen::Vector3d(point.getFieldAs<double>(DimId::X),
                                        point.getFieldAs<double>(DimId::Y),
                                        point.getFieldAs<double>(DimId::Z)));
        if (m_config->m_ned)
        {
            point.setField(DimId::X, ned.y());
            point.setField(DimId::Y, ned.x());
            point.setField(DimId::Z, -ned.z());
        }
        else
        {
            point.setField(DimId::X, ned.x());
            point.setField(DimId::Y, ned.y());
            point.setField(DimId::Z, ned.z());
        }
        m_localCartesian->reverse(point);

        // Transform BeamOrigin (point) - same as X/Y/Z
        if (m_config->m_transformBeam)
        {
            // Apply same transformation as for X/Y/Z
            const Eigen::Vector3d nedOrig(
                transform *
                Eigen::Vector3d(point.getFieldAs<double>(DimId::BeamOriginX),
                                point.getFieldAs<double>(DimId::BeamOriginY),
                                point.getFieldAs<double>(DimId::BeamOriginZ)));

            const Eigen::Vector3d nedDir(
                transform *
                Eigen::Vector3d(
                    point.getFieldAs<double>(DimId::BeamOriginX) +
                        point.getFieldAs<double>(DimId::BeamDirectionX),
                    point.getFieldAs<double>(DimId::BeamOriginY) +
                        point.getFieldAs<double>(DimId::BeamDirectionY),
                    point.getFieldAs<double>(DimId::BeamOriginZ) +
                        point.getFieldAs<double>(DimId::BeamDirectionZ)));
            double originX(nedOrig.x()), originY(nedOrig.y()),
                originZ(nedOrig.z());
            double directionX(nedDir.x()), directionY(nedDir.y()),
                directionZ(nedDir.z());
            if (m_config->m_ned)
            {
                directionX = nedDir.y();
                directionY = nedDir.x();
                directionZ = -nedDir.z();
                originX = nedOrig.y();
                originY = nedOrig.x();
                originZ = -nedOrig.z();
            }
            m_localCartesian->reverse(originX, originY, originZ);
            m_localCartesian->reverse(directionX, directionY, directionZ);
            point.setField(DimId::BeamOriginX, originX);
            point.setField(DimId::BeamOriginY, originY);
            point.setField(DimId::BeamOriginZ, originZ);

            const Eigen::Vector3d normalized =
                Eigen::Vector3d(directionX - originX, directionY - originY,
                                directionZ - originZ)
                    .normalized();
            point.setField(DimId::BeamDirectionX, normalized.x());
            point.setField(DimId::BeamDirectionY, normalized.y());
            point.setField(DimId::BeamDirectionZ, normalized.z());
        }
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
