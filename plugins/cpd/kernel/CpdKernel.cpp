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

#include "kernel/CpdKernel.hpp"
#include <pdal/pdal_macros.hpp>

#include <cpd/nonrigid.hpp>
#include <cpd/rigid.hpp>

#include <filters/CropFilter.hpp>
#include <io/BufferReader.hpp>
#include <pdal/KernelFactory.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/EigenUtils.hpp>

namespace pdal {

static PluginInfo const s_info = PluginInfo(
    "kernels.cpd", "CPD Kernel", "http://pdal.io/kernels/kernels.cpd.html");

CREATE_SHARED_PLUGIN(1, 0, CpdKernel, Kernel, s_info)

std::string CpdKernel::getName() const { return s_info.name; }

void CpdKernel::addSwitches(ProgramArgs& args) {
    Arg& method =
        args.add("method,M", "registration method (rigid, nonrigid)", m_method);
    method.setPositional();
    Arg& fixed =
        args.add("fixed,f", "input file containing the fixed points", m_fixed);
    fixed.setPositional();
    Arg& moving = args.add("moving,m",
                          "input file containing the moving points, "
                          "i.e. the points that will be registered",
                          m_moving);
    moving.setPositional();
    Arg& output = args.add("output,o", "output file name", m_output);
    output.setPositional();
    args.add("bounds", "Extent (in XYZ) to clip output to", m_bounds);

    args.add("max-iterations", "maximum number of iterations allowed",
             m_max_iterations, cpd::DEFAULT_MAX_ITERATIONS);
    args.add("normalize", "whether cpd should normalize the points before running",
             m_normalize, true);
    args.add("outliers", "a number between zero and one that represents the tolerance for outliers",
             m_outliers, cpd::DEFAULT_OUTLIERS);
    args.add("sigma2", "the starting sigma2 value.",
             m_sigma2, cpd::DEFAULT_SIGMA2);
    args.add("tolerance", "the amount the error must change to continue iterations",
             m_tolerance, cpd::DEFAULT_TOLERANCE);

    args.add("reflections", "should rigid registrations allow reflections",
            m_reflections, false);
    args.add("scale", "should rigid registrations allow scaling",
            m_reflections, false);

    args.add("beta", "beta parameter for nonrigid registrations",
            m_beta, cpd::DEFAULT_BETA);
    args.add("lambda", "lambda parameter for nonrigid registrations",
            m_lambda, cpd::DEFAULT_LAMBDA);
}

cpd::Matrix CpdKernel::readFile(const std::string& filename) {
    Stage& reader = makeReader(filename, "");

    PointTable table;
    PointViewSet viewSet;
    if (!m_bounds.empty()) {
        Options boundsOptions;
        boundsOptions.add("bounds", m_bounds);

        Stage& crop = makeFilter("filters.crop", reader);
        crop.setOptions(boundsOptions);
        crop.prepare(table);
        viewSet = crop.execute(table);
    } else {
        reader.prepare(table);
        viewSet = reader.execute(table);
    }

    return eigen::pointViewToEigen(**viewSet.begin());
}

int CpdKernel::execute() {
    cpd::Matrix fixed = readFile(m_fixed);
    cpd::Matrix moving = readFile(m_moving);

    if (fixed.rows() == 0 || moving.rows() == 0) {
        throw pdal_error("No points to process.");
    }

    cpd::Matrix result;
    if (m_method == "rigid") {
        cpd::Rigid rigid;
        rigid.max_iterations(m_max_iterations)
            .normalize(m_normalize)
            .outliers(m_outliers)
            .sigma2(m_sigma2)
            .tolerance(m_tolerance);
        rigid.reflections(m_reflections)
            .scale(m_scale);
        result = rigid.run(fixed, moving).points;
    } else if (m_method == "nonrigid") {
        cpd::Nonrigid nonrigid;
        nonrigid.max_iterations(m_max_iterations)
            .normalize(m_normalize)
            .outliers(m_outliers)
            .sigma2(m_sigma2)
            .tolerance(m_tolerance);
        nonrigid.beta(m_beta).lambda(m_lambda);
        result = nonrigid.run(fixed, moving).points;
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

    size_t rows = moving.rows();
    for (size_t i = 0; i < rows; ++i) {
        outView->setField<double>(Dimension::Id::X, i, result(i, 0));
        outView->setField<double>(Dimension::Id::Y, i, result(i, 1));
        outView->setField<double>(Dimension::Id::Z, i, result(i, 2));
        outView->setField<double>(Dimension::Id::XVelocity, i,
                                  moving(i, 0) - result(i, 0));
        outView->setField<double>(Dimension::Id::YVelocity, i,
                                  moving(i, 1) - result(i, 1));
        outView->setField<double>(Dimension::Id::ZVelocity, i,
                                  moving(i, 2) - result(i, 2));
    }

    BufferReader reader;
    reader.addView(outView);

    Options writerOpts;
    if (StageFactory::inferReaderDriver(m_output) == "writers.text") {
        writerOpts.add("order", "X,Y,Z,XVelocity,YVelocity,ZVelocity");
        writerOpts.add("keep_unspecified", false);
    }
    Stage& writer = makeWriter(m_output, reader, "", writerOpts);
    writer.prepare(outTable);
    writer.execute(outTable);

    return 0;
}

}  // namespace pdal
