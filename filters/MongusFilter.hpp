/******************************************************************************
* Copyright (c) 2016, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <Eigen/Dense>

#include <memory>
#include <unordered_map>

namespace pdal
{

class PointLayout;
class PointView;

typedef std::unordered_map<int, std::vector<PointId>> PointIdHash;

class PDAL_DLL MongusFilter : public Filter
{
public:
    MongusFilter() : Filter()
    {}

    std::string getName() const;

private:
    bool m_classify;
    bool m_extract;
    int m_numRows;
    int m_numCols;
    int m_maxRow;
    double m_cellSize;
    double m_k;
    int m_l;
    BOX2D m_bounds;

    virtual void addDimensions(PointLayoutPtr layout);
    virtual void addArgs(ProgramArgs& args);
    int getColIndex(double x, double cell_size);
    int getRowIndex(double y, double cell_size);
    void writeControl(Eigen::MatrixXd cx, Eigen::MatrixXd cy, Eigen::MatrixXd cz, std::string filename);
    void downsampleMin(Eigen::MatrixXd *cx, Eigen::MatrixXd *cy,
                       Eigen::MatrixXd* cz, Eigen::MatrixXd *dcx,
                       Eigen::MatrixXd *dcy, Eigen::MatrixXd* dcz,
                       double cell_size);
    std::vector<PointId> processGround(PointViewPtr view);
    virtual PointViewSet run(PointViewPtr view);

    MongusFilter& operator=(const MongusFilter&); // not implemented
    MongusFilter(const MongusFilter&); // not implemented
};

} // namespace pdal
