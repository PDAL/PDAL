/******************************************************************************
 * Copyright (c) 2014, Pete Gadomski (pete.gadomski@gmail.com)
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

#include "kernel/Cpd.hpp"
#include <pdal/pdal_macros.hpp>

#include <cpd/rigid.hpp>
#include <cpd/nonrigid.hpp>

#include <pdal/KernelFactory.hpp>
#include <pdal/StageFactory.hpp>

#include "chipper/ChipperFilter.hpp"
#include "crop/CropFilter.hpp"
#include "buffer/BufferReader.hpp"

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
                                     "kernels.cpd",
                                     "CPD Kernel",
                                     "http://pdal.io/kernels/kernels.cpd.html" );

CREATE_SHARED_PLUGIN(1, 0, CpdKernel, Kernel, s_info)

std::string CpdKernel::getName() const {
    return s_info.name;
}

void CpdKernel::addSwitches(ProgramArgs& args)
{
    Arg& method = args.add("method,M", "registration method (rigid, nonrigid)",
                           m_method);
    method.setPositional();
    Arg& filex = args.add("filex,x", "input file containing the source points",
                          m_filex);
    filex.setPositional();
    Arg& filey = args.add("filey,y", "input file containg target points, "
                          "i.e. the points that will be registered", m_filey);
    filey.setPositional();
    Arg& output = args.add("output,o", "output file name", m_output);
    output.setPositional();
    args.add("tolerance,t", "tolerance criterium", m_tolerance,
             cpd::Rigid::DEFAULT_TOLERANCE);
    args.add("max-iterations,m", "maximum number of iterations allowed",
             m_max_it, cpd::Rigid::DEFAULT_MAX_ITERATIONS);
    args.add("outliers,O", "the weight of noise and outliers",
             m_outliers, cpd::Rigid::DEFAULT_OUTLIER_WEIGHT);
    args.add("no-reflections,r", "Prevent reflections of the data",
             m_no_reflections, true);
    args.add("allow-scaling,S", "Allow scaling of the data",
             m_allow_scaling, false);
    args.add("beta,b", "std of gaussian filter (Green's function, used "
             "for nonrigid registrations only)", m_beta, cpd::Nonrigid::DEFAULT_BETA);
    args.add("lambda,l", "regularization weight (used for nonrigid "
             "registrations only)", m_lambda, cpd::Nonrigid::DEFAULT_LAMBDA);
    args.add("bounds", "Extent (in XYZ) to clip output to", m_bounds);
    args.add("sigma2",
             "The starting sigma2 value. To improve CPD runs, set to a bit "
             "more than you expect the average motion to be",
             m_sigma2, 0.0);
}


cpd::Matrix CpdKernel::readFile(const std::string& filename)
{
    Stage& reader = makeReader(filename, "");

    PointTable table;
    PointViewSet viewSet;
    if (!m_bounds.empty())
    {
        Options boundsOptions;
        boundsOptions.add("bounds", m_bounds);

        Stage& crop = makeFilter("filters.crop", reader);
        crop.setOptions(boundsOptions);
        crop.prepare(table);
        viewSet = crop.execute(table);
    }
    else
    {
        reader.prepare(table);
        viewSet = reader.execute(table);
    }

    cpd::Matrix matrix(0, 3);
    for (auto it = viewSet.begin(); it != viewSet.end(); ++it)
    {
        PointViewPtr view = *it;
        point_count_t rowidx;
        if (matrix.rows() == 0)
        {
            rowidx = 0;
            matrix.resize(view->size(), 3);
        }
        else
        {
            rowidx = matrix.rows();
            matrix.conservativeResize(matrix.rows() + view->size(), 3);
        }

        for (point_count_t bufidx = 0; bufidx < view->size(); ++bufidx, ++rowidx)
        {
            matrix(rowidx, 0) = view->getFieldAs<double>(Dimension::Id::X, bufidx);
            matrix(rowidx, 1) = view->getFieldAs<double>(Dimension::Id::Y, bufidx);
            matrix(rowidx, 2) = view->getFieldAs<double>(Dimension::Id::Z, bufidx);
        }
    }
    return matrix;
}


int CpdKernel::execute()
{
    PointTable tableX;
    PointTable tableY;

    cpd::Matrix X = readFile(m_filex);
    cpd::Matrix Y = readFile(m_filey);

    if (X.rows() == 0 || Y.rows() == 0)
    {
        throw pdal_error("No points to process.");
    }

    cpd::Matrix result;
    if (m_method == "rigid") {
        cpd::Rigid rigid;
        rigid
            .set_tolerance(m_tolerance)
            .set_max_iterations(m_max_it)
            .set_outlier_weight(m_outliers);
        rigid
            .no_reflections(m_no_reflections)
            .allow_scaling(m_allow_scaling);
        if (m_sigma2 > 0) {
            result = rigid.compute(X, Y, m_sigma2).points;
        } else {
            result = rigid.compute(X, Y).points;
        }
    } else if (m_method == "nonrigid") {
        cpd::Nonrigid nonrigid;
        nonrigid
            .set_tolerance(m_tolerance)
            .set_max_iterations(m_max_it)
            .set_outlier_weight(m_outliers);
        nonrigid
            .set_beta(m_beta)
            .set_lambda(m_lambda);
        if (m_sigma2 > 0) {
            result = nonrigid.compute(X, Y, m_sigma2).points;
        } else {
            result = nonrigid.compute(X, Y).points;
        }
    } else {
        std::stringstream ss;
        ss << "Invalid cpd method: " << m_method << std::endl;
        throw pdal_error(ss.str());
    }

    PointTable outTable;
    PointLayoutPtr outLayout(outTable.layout());
    outLayout->registerDim(Dimension::Id::X);
    outLayout->registerDim(Dimension::Id::Y);
    outLayout->registerDim(Dimension::Id::Z);
    outLayout->registerDim(Dimension::Id::XVelocity);
    outLayout->registerDim(Dimension::Id::YVelocity);
    outLayout->registerDim(Dimension::Id::ZVelocity);
    PointViewPtr outView(new PointView(outTable));

    size_t M = Y.rows();
    for (size_t i = 0; i < M; ++i)
    {
        outView->setField<double>(Dimension::Id::X, i, result(i, 0));
        outView->setField<double>(Dimension::Id::Y, i, result(i, 1));
        outView->setField<double>(Dimension::Id::Z, i, result(i, 2));
        outView->setField<double>(Dimension::Id::XVelocity, i,
                                  Y(i, 0) - result(i, 0));
        outView->setField<double>(Dimension::Id::YVelocity, i,
                                  Y(i, 1) - result(i, 1));
        outView->setField<double>(Dimension::Id::ZVelocity, i,
                                  Y(i, 2) - result(i, 2));
    }

    BufferReader reader;
    reader.addView(outView);

    Options writerOpts;
    if (StageFactory::inferReaderDriver(m_output) == "writers.text")   
    {
        writerOpts.add("order", "X,Y,Z,XVelocity,YVelocity,ZVelocity");
        writerOpts.add("keep_unspecified", false);
    }
    Stage& writer = makeWriter(m_output, reader, "", writerOpts);
    writer.prepare(outTable);
    writer.execute(outTable);

    return 0;
}

} // namespace pdal
