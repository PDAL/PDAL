/******************************************************************************
 * Copyright (c) 2017, Peter J. Gadomski <pete@gadom.ski>
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

#include <cpd/affine.hpp>
#include <cpd/nonrigid.hpp>
#include <cpd/rigid.hpp>
#include <filters/CpdFilter.hpp>
#include <pdal/private/MathUtils.hpp>

namespace pdal
{
namespace
{
void movePoints(PointViewPtr moving, const cpd::Matrix& result)
{
    assert(moving->size() == (point_count_t)result.rows());
    for (PointId i = 0; i < moving->size(); ++i)
    {
        moving->setField(Dimension::Id::X, i, result(i, 0));
        moving->setField(Dimension::Id::Y, i, result(i, 1));
        moving->setField(Dimension::Id::Z, i, result(i, 2));
    }
}

void addMetadata(CpdFilter* filter, const cpd::Result& result)
{
    MetadataNode root = filter->getMetadata();
    root.add("sigma2", result.sigma2);
    root.add("runtime", double(result.runtime.count()) / 1e6);
    root.add("iterations", result.iterations);
}
}

static PluginInfo const s_info
{
    "filters.cpd",
    "CPD filter",
    "http://pdal.io/stages/filters.cpd.html"
};

CREATE_SHARED_STAGE(CpdFilter, s_info)

std::string CpdFilter::getName() const
{
    return s_info.name;
}

void CpdFilter::addArgs(ProgramArgs& args)
{
    args.add("method", "CPD method (rigid, nonrigid, or affine)", m_method,
             "rigid");
}

std::string CpdFilter::defaultMethod()
{
    return "rigid";
}

PointViewSet CpdFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;
    if (this->m_complete)
    {
        throw pdal_error(
            "filters.cpd must have two point view inputs, no more, no less");
    }
    else if (this->m_fixed)
    {
        log()->get(LogLevel::Debug2) << "Adding moving points\n";
        PointViewPtr result = this->change(this->m_fixed, view);
        viewSet.insert(result);
        this->m_complete = true;
        log()->get(LogLevel::Debug2) << "filters.cpd complete\n";
    }
    else
    {
        log()->get(LogLevel::Debug2) << "Adding fixed points\n";
        this->m_fixed = view;
    }
    return viewSet;
}

void CpdFilter::done(PointTableRef _)
{
    if (!this->m_complete)
    {
        throw pdal_error(
            "filters.cpd must have two point view inputs, no more, no less");
    }
}

PointViewPtr CpdFilter::change(PointViewPtr fixed, PointViewPtr moving)
{
    MetadataNode root = this->getMetadata();
    root.add("method", this->m_method);
    log()->get(LogLevel::Debug2)
        << "filters.cpd running method:" << this->m_method << "\n";
    if (this->m_method == "rigid")
    {
        this->cpd_rigid(fixed, moving);
    }
    else if (this->m_method == "affine")
    {
        this->cpd_affine(fixed, moving);
    }
    else if (this->m_method == "nonrigid")
    {
        this->cpd_nonrigid(fixed, moving);
    }
    else
    {
        std::stringstream ss;
        ss << "Invalid cpd detection method: " << this->m_method;
        throw pdal_error(ss.str());
    }
    return moving;
}

namespace
{

cpd::Matrix pointViewToEigen(const PointViewPtr *view)
{
    Eigen::MatrixXd matrix(view->size(), 3);
    for (PointId i = 0; i < view->size(); ++i)
    {
        matrix(i, 0) = view->getFieldAs<double>(Dimension::Id::X, i);
        matrix(i, 1) = view->getFieldAs<double>(Dimension::Id::Y, i);
        matrix(i, 2) = view->getFieldAs<double>(Dimension::Id::Z, i);
    }
    return matrix;
}

} // unnamed namespace

void CpdFilter::cpd_rigid(PointViewPtr fixed, PointViewPtr moving)
{
    cpd::Matrix fixedMatrix = pointViewToEigen(fixed);
    cpd::Matrix movingMatrix = pointViewToEigen(moving);
    cpd::RigidResult result = cpd::rigid(fixedMatrix, movingMatrix);
    movePoints(moving, result.points);
    addMetadata(this, static_cast<cpd::Result>(result));
    MetadataNode root = getMetadata();
    root.add("transform", result.matrix());
}

void CpdFilter::cpd_affine(PointViewPtr fixed, PointViewPtr moving)
{
    cpd::Matrix fixedMatrix = pointViewToEigen(fixed);
    cpd::Matrix movingMatrix = pointViewToEigen(moving);
    cpd::AffineResult result = cpd::affine(fixedMatrix, movingMatrix);
    movePoints(moving, result.points);
    MetadataNode root = getMetadata();
    root.add("transform", result.matrix());
}

void CpdFilter::cpd_nonrigid(PointViewPtr fixed, PointViewPtr moving)
{
    cpd::Matrix fixedMatrix = math::pointViewToEigen(*fixed);
    cpd::Matrix movingMatrix = math::pointViewToEigen(*moving);
    cpd::NonrigidResult result = cpd::nonrigid(fixedMatrix, movingMatrix);
    movePoints(moving, result.points);
}
}
