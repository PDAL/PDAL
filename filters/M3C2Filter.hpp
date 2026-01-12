/******************************************************************************
* Copyright (c) 2025, Hobu Inc.
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

#include <Eigen/Dense>

#include <pdal/Filter.hpp>
#include <pdal/KDIndex.hpp>

namespace pdal
{

class PointView;

class PDAL_EXPORT M3C2Filter : public Filter
{
    struct Stats
    {
        double distance;
        double uncertainty;
        double significant;
        double stdDev1;
        double stdDev2;
        int n1;
        int n2;
    };

    enum class NormalOrientation
    {
        Up,
        Origin,
        None
    };

    friend std::istream& operator>>(std::istream& in,
        M3C2Filter::NormalOrientation& mode);
    friend std::ostream& operator<<(std::ostream& in,
        const M3C2Filter::NormalOrientation& mode);

    struct Args;
    struct Private;

public:
    M3C2Filter();
    ~M3C2Filter();
    M3C2Filter& operator=(const M3C2Filter&) = delete;
    M3C2Filter(const M3C2Filter&) = delete;

    std::string getName() const;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void initialize();
    virtual void prerun(const PointViewSet& pvSet);
    virtual PointViewSet run(PointViewPtr view);
    virtual void done(PointTableRef table);

    void createSample(PointView& source, PointView& dest);
    void calcStats(PointView& v1, PointView& v2, PointView& cores);
    bool calcStats(Eigen::Vector3d cylCenter, Eigen::Vector3d cylNormal,
        PointView& v1, PointView& v2, Stats& stats);
    KD3Index::RadiusResults filterPoints(Eigen::Vector3d cylCenter, Eigen::Vector3d cylNormal,
        const KD3Index::RadiusResults& ids, const PointView& view);
    double pointPasses(Eigen::Vector3d point, Eigen::Vector3d cylCenter, Eigen::Vector3d cylNormal);

    std::unique_ptr<M3C2Filter::Args> m_args;

    std::unique_ptr<M3C2Filter::Private> m_p;
};

} // namespace pdal
