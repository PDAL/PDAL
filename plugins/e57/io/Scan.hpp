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

#pragma once

#include <pdal/pdal_types.hpp>
#include <pdal/Writer.hpp>
#include <pdal/Dimension.hpp>
#include <E57Format.h>

namespace e57
{
class Scan
{

public:
    Scan(const e57::StructureNode& scanNode);

    pdal::point_count_t getNumPoints() const;

    /// Get the pdal dimensions that can be read from this scan
    std::set<std::string> getDimensions() const;

    e57::CompressedVectorNode getPoints() const;

    std::pair<double,double> getLimits(pdal::Dimension::Id pdalId) const;

    bool hasPose() const;
    void transformPoint(pdal::PointRef pt) const;
    pdal::BOX3D getBoundingBox() const;

private:
    /// Called only once on constructor called
    void decodeHeader();

    void getPose();

    // Core data holders for underlying e57 object
    std::unique_ptr<e57::StructureNode> m_rawData;
    std::unique_ptr<e57::CompressedVectorNode> m_rawPoints;
    pdal::point_count_t m_numPoints;
    std::array<double,3>
        transformPoint(const  std::array<double,3> &originalPoint) const;

    // supported configs
    std::set<std::string> m_e57TypeToPdalDimension;

    // field limits in header
    std::map<pdal::Dimension::Id,std::pair<double,double>> m_valueBounds;

    // Pose information
    double m_translation[3] = {0};
    double m_rotation[3][3] = {{0}};
    bool m_hasPose = false;
    pdal::BOX3D m_bbox;
};
}
