/******************************************************************************
* Copyright (c) 2019, Helix Re Inc.
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
*     * Neither the name of Helix Re Inc. nor the
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

#include "Scan.hpp"
#include "Utils.hpp"

namespace e57
{
Scan::Scan(const e57::StructureNode &node) : m_numPoints(0)
{
    m_rawData = std::unique_ptr<e57::StructureNode>(new e57::StructureNode(node));
    m_rawPoints = std::unique_ptr<CompressedVectorNode>(new CompressedVectorNode(m_rawData->get("points")));
    decodeHeader();
}

pdal::point_count_t Scan::getNumPoints() const
{
    return m_numPoints;
}

std::set<std::string> Scan::getDimensions() const
{
    return m_e57TypeToPdalDimension;
}

e57::CompressedVectorNode Scan::getPoints() const
{
    return *m_rawPoints;
}


bool Scan::hasPose() const
{
    return m_hasPose;
}

void Scan::transformPoint(pdal::PointRef pt) const
{
    auto x = pt.getFieldAs<double>(pdal::Dimension::Id::X);
    auto y = pt.getFieldAs<double>(pdal::Dimension::Id::Y);
    auto z = pt.getFieldAs<double>(pdal::Dimension::Id::Z);

    pt.setField(pdal::Dimension::Id::X, x*m_rotation[0][0] + y*m_rotation[0][1] + z*m_rotation[0][2] + m_translation[0]);
    pt.setField(pdal::Dimension::Id::Y,  x*m_rotation[1][0] + y*m_rotation[1][1] + z*m_rotation[1][2]  + m_translation[1]);
    pt.setField(pdal::Dimension::Id::Z,  x*m_rotation[2][0] + y*m_rotation[2][1] + z*m_rotation[2][2]  + m_translation[2]);
}

std::array<double,3>
Scan::transformPoint(const std::array<double,3> &originalPoint) const
{
    std::array<double,3> transformed {0,0,0};
    for (size_t i = 0; i < originalPoint.size(); i++)
    {
        transformed[i] =  m_translation[i];
        for (size_t  j = 0; j < originalPoint.size(); j++)
            transformed[i] += originalPoint[i]*m_rotation[i][j];
    }
    return transformed;
}

pdal::BOX3D Scan::getBoundingBox() const
{
    if (!hasPose())
    {
        return m_bbox;
    }

    auto bmin = transformPoint({m_bbox.minx,m_bbox.miny,m_bbox.minz});
    auto bmax = transformPoint({m_bbox.maxx,m_bbox.maxy,m_bbox.maxz});
    return pdal::BOX3D(bmin[0],bmin[1],bmin[2],bmax[0],bmax[1],bmax[2]);
}

void Scan::decodeHeader()
{
    m_numPoints = m_rawPoints->childCount();

    auto supportedFields = pdal::e57plugin::supportedE57Types();
    e57::StructureNode prototype(m_rawPoints->prototype());

    // Extract fields that can be extracted
    for (auto& field: supportedFields)
    {
        if (prototype.isDefined(field))
        {
            m_e57TypeToPdalDimension.insert(field);
        }
    }
    // Get pose estimation
    getPose();

    // Get rescale factors for new scan, These factors are only for
    // dimensions with limits. This is required Since,
    // 1. E57 support colors in uint8 as well as uint16. This is specified
    //    in header as "colorLimits".
    //      - If colorLimits is 0-255 then it is uint8 and if 0-65535 then
    // it
    // is uint16.
    //      - To make this consistant, We are rescaling colors to 0-65535
    //        range, Since default types for colors in PDAL are Unsigned16.
    // 2. E57 supports intensity in float ranging 0-1.  This is specified in
    //    header as "intensityLimits".
    //      - Since default type for intensity in PDAL is Unsigned16, E57
    //        intensity (between 0-1) need to be rescaled in uint16 (between
    //        0-65535).
    // To do the rescaling we need the rescale factors so that we can
    // directly multiply them with colors and intensity values. E.g. - If
    // color limit is 0-255 then rescale factor would be 257.00
    //        (double value) i.e 65535/(255-0)=257.
    //      - If color limit is 0-65535 then rescale factor would be 1.00
    //        (double value) i.e 65535/(65535-0)= 1.

    std::fill_n(m_rescaleFactors, pdal::Dimension::COUNT, 1.0f);

    auto scalableFields = pdal::e57plugin::scalableE57Types();
    for (auto& field : scalableFields)
    {
        auto minmax = std::make_pair(0.0, 0.0);
        if (pdal::e57plugin::getLimits(*m_rawData, field, minmax))
        {
            auto dim = pdal::e57plugin::e57ToPdal(field);
            m_rescaleFactors[(int)dim] =
                pdal::e57plugin::getPdalBounds(dim).second / (minmax.second - minmax.first);
        }
    }

    // Cartesian Bounds
    auto minMaxx = std::make_pair(0.0, 0.0);
    auto minMaxy = std::make_pair(0.0, 0.0);
    auto minMaxz = std::make_pair(0.0, 0.0);
    pdal::e57plugin::getLimits(*m_rawData, "x", minMaxx);
    pdal::e57plugin::getLimits(*m_rawData, "y", minMaxy);
    pdal::e57plugin::getLimits(*m_rawData, "z", minMaxz);
    m_bbox.grow(minMaxx.first, minMaxy.first, minMaxz.first);
    m_bbox.grow(minMaxx.second, minMaxy.second, minMaxz.second);
}

void Scan::getPose()
{
    if (m_rawData->isDefined("pose"))
    {
        // Reset rotation and translation.
        m_rotation[0][0] = 1;
        m_rotation[0][1] = 0;
        m_rotation[0][2] = 0;
        m_rotation[1][0] = 0;
        m_rotation[1][1] = 1;
        m_rotation[1][2] = 0;
        m_rotation[2][0] = 0;
        m_rotation[2][1] = 0;
        m_rotation[2][2] = 1;
        m_translation[0] = 0;
        m_translation[1] = 0;
        m_translation[2] = 0;

        m_hasPose = true;

        e57::StructureNode pose(m_rawData->get("pose"));
        if (pose.isDefined("rotation"))
        {
            e57::StructureNode rotNode(pose.get("rotation"));
            double q[4];
            q[0] = e57::FloatNode(rotNode.get("w")).value();
            q[1] = e57::FloatNode(rotNode.get("x")).value();
            q[2] = e57::FloatNode(rotNode.get("y")).value();
            q[3] = e57::FloatNode(rotNode.get("z")).value();

            double q11 = q[1] * q[1];
            double q22 = q[2] * q[2];
            double q33 = q[3] * q[3];
            double q03 = q[0] * q[3];
            double q13 = q[1] * q[3];
            double q23 = q[2] * q[3];
            double q02 = q[0] * q[2];
            double q12 = q[1] * q[2];
            double q01 = q[0] * q[1];

            m_rotation[0][0] = 1 - 2.0*(q22 + q33);
            m_rotation[1][1] = 1 - 2.0*(q11 + q33);
            m_rotation[2][2] = 1 - 2.0*(q11 + q22);
            m_rotation[0][1] = 2.0*(q12 - q03);
            m_rotation[1][0] = 2.0*(q12 + q03);
            m_rotation[0][2] = 2.0*(q13 + q02);
            m_rotation[2][0] = 2.0*(q13 - q02);
            m_rotation[1][2] = 2.0*(q23 - q01);
            m_rotation[2][1] = 2.0*(q23 + q01);
        }

        if (pose.isDefined("translation"))
        {
            e57::StructureNode transNode(pose.get("translation"));
            m_translation[0] = e57::FloatNode(transNode.get("x")).value();
            m_translation[1] = e57::FloatNode(transNode.get("y")).value();
            m_translation[2] = e57::FloatNode(transNode.get("z")).value();
        }
    }
    else
        m_hasPose = false;
}

double Scan::rescale(pdal::Dimension::Id dim, double value)
{
    return m_rescaleFactors[(int)dim] * value;
}

StructureNode Scan::getPointPrototype()
{
    return StructureNode(getPoints().prototype());
}
}
