/******************************************************************************
* Copyright (c) 2016-2017, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/Filter.hpp>
#include <pdal/plugin.hpp>

#include <Eigen/Dense>

#include <string>

extern "C" int32_t SMRFilter_ExitFunc();
extern "C" PF_ExitFunc SMRFilter_InitPlugin();

namespace pdal
{

class PDAL_DLL SMRFilter : public Filter
{
public:
    SMRFilter() : Filter()
    {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

private:
    int m_rows;
    int m_cols;
    double m_cell;
    double m_cut;
    double m_slope;
    double m_window;
    double m_scalar;
    double m_threshold;
    std::string m_dir;
    BOX2D m_bounds;
    SpatialReference m_srs;

    virtual void addArgs(ProgramArgs& args);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual PointViewSet run(PointViewPtr view);

    void classifyGround(PointViewPtr, Eigen::MatrixXd const&);
    Eigen::MatrixXi createLowMask(Eigen::MatrixXd const&);
    Eigen::MatrixXi createNetMask();
    Eigen::MatrixXi createObjMask(Eigen::MatrixXd const&);
    Eigen::MatrixXd createZImin(PointViewPtr);
    Eigen::MatrixXd createZInet(Eigen::MatrixXd const&, Eigen::MatrixXi const&);
    Eigen::MatrixXd createZIpro(PointViewPtr, Eigen::MatrixXd const&,
                                Eigen::MatrixXi const&, Eigen::MatrixXi const&,
                                Eigen::MatrixXi const&);
    Eigen::MatrixXd knnfill(PointViewPtr, Eigen::MatrixXd const&);
    Eigen::MatrixXi progressiveFilter(Eigen::MatrixXd const&, double, double);

    SMRFilter& operator=(const SMRFilter&); // not implemented
    SMRFilter(const SMRFilter&); // not implemented
};

} // namespace pdal
