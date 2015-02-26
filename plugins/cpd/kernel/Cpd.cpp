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

#include <pdal/BufferReader.hpp>
#include <pdal/KernelFactory.hpp>

#include "chipper/ChipperFilter.hpp"
#include "crop/CropFilter.hpp"

namespace pdal
{

static PluginInfo const s_info {
    "kernels.cpd",
    "CPD Kernel",
    "http://pdal.io/kernels/kernels.cpd.html" };

CREATE_SHARED_PLUGIN(CpdKernel, Kernel, s_info)


void CpdKernel::validateSwitches()
{
    if (m_filex == "")
        throw app_usage_error("--filex/-x required");
    if (m_filey == "")
        throw app_usage_error("--filey/-y required");
    if (m_output == "")
        throw app_usage_error("--output/-o required");
}


void CpdKernel::addSwitches()
{
    using namespace cpd;

    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
        ("filex,x", po::value<std::string>(&m_filex)->default_value(""),
         "input file containing the source points")
        ("filey,y", po::value<std::string>(&m_filey)->default_value(""),
         "input file containg target points, i.e. the points that will be registered")
        ("output,o", po::value<std::string>(&m_output)->default_value(""),
         "output file name")
        ("tolerance,t", po::value<float>(&m_tolerance)->default_value(DefaultTolerance),
            "tolerance criterium")
        ("max-iterations,m", po::value<int>(&m_max_it)->default_value(DefaultMaxIterations),
            "maximum number of iterations allowed")
        ("outliers,O", po::value<float>(&m_outliers)->default_value(DefaultOutliers),
            "the weight of noise and outliers")
        ("fgt,f", po::value<bool>(&m_fgt)->default_value(DefaultFgt),
            "use a fast gauss transform (less accurate but faster)")
        ("epsilon,e", po::value<float>(&m_epsilon)->default_value(DefaultEpsilon),
            "tolerance level for the fast gauss transform")
        ("beta,b", po::value<float>(&m_beta)->default_value(DefaultBeta),
            "std of gaussian filter (Green's function, used for nonrigid registrations only)")
        ("lambda,l", po::value<float>(&m_lambda)->default_value(DefaultLambda),
            "regularization weight (used for nonrigid registrations only)")
        ("numeig,n", po::value<arma::uword>(&m_numeig)->default_value(DefaultNumeig),
            "number of eigenvectors/eigenvalues to find for nonrigid_lowrank registrations (if zero, default to N ^ (1/2) where N is the number of points in Y)")
        ("bounds", po::value<BOX3D>(&m_bounds),
            "Extent (in XYZ) to clip output to")
        ("auto-z-exaggeration", po::value<bool>(&m_auto_z_exaggeration)->default_value(false),
            "Use the domain of the XY dimensions to automatically exaggerate the Z dimensions")
        ("auto-z-exaggeration-ratio", po::value<float>(&m_auto_z_exaggeration_ratio)->default_value(5.0 / 8.0),
            "The scaling ratio for the Z-exaggeration. Z's range will be scaled to this ratio of the extent of the smallest XY extent.")
        ("chipped", po::value<bool>(&m_chipped)->default_value(false),
            "Run chipped registration")
        ("chip-capacity", po::value<int>(&m_chip_capacity)->default_value(8000),
            "The maximum number of points in each chip (before buffer)")
        ("chip-buffer", po::value<float>(&m_chip_buffer)->default_value(50),
            "The width of the buffer around each chip")
        ("sigma2", po::value<float>(&m_sigma2)->default_value(DefaultSigma2),
            "The starting sigma2 value. To improve CPD runs, set to a bit more than you expect the average motion to be")
        ;

    addSwitchSet(file_options);

    addPositionalSwitch("filex", 1);
    addPositionalSwitch("filey", 1);
    addPositionalSwitch("output", 1);
}


PointBufferPtr CpdKernel::readFile(const std::string& filename, PointContext& ctx, arma::mat& mat)
{
    Options opt;
    opt.add<std::string>("filename", filename);
    opt.add<bool>("debug", isDebug());
    opt.add<boost::uint32_t>("verbose", getVerboseLevel());

    std::shared_ptr<Stage> reader(KernelSupport::makeReader(filename));
    reader->setOptions(opt);

    std::shared_ptr<Stage> source(reader);

    if (!m_bounds.empty())
    {
        Options boundsOptions;
        boundsOptions.add("bounds", m_bounds);
        StageFactory f;
        std::shared_ptr<Stage> crop(f.createStage("filters.crop"));
        crop->setInput(source.get());
        crop->setOptions(boundsOptions);
        source = crop;
    }

    source->prepare(ctx);
    PointBufferSet pbset = source->execute(ctx);

    const arma::uword D = 3;
    for (auto it = pbset.begin(); it != pbset.end(); ++it)
    {
        point_count_t rowidx;
        if (mat.is_empty())
        {
            rowidx = 0;
            mat.set_size((*it)->size(), D);
        }
        else
        {
            rowidx = mat.n_rows;
            mat.set_size(mat.n_rows + (*it)->size(), D);
        }

        for (point_count_t bufidx = 0; bufidx < (*it)->size(); ++bufidx, ++rowidx)
        {
            mat(rowidx, 0) = (*it)->getFieldAs<double>(Dimension::Id::X, bufidx);
            mat(rowidx, 1) = (*it)->getFieldAs<double>(Dimension::Id::Y, bufidx);
            mat(rowidx, 2) = (*it)->getFieldAs<double>(Dimension::Id::Z, bufidx);
        }
    }
    // Return a pointer to the first point buffer because we assume
    // that readers only produce one point buffer. If that assumption
    // ever is invalid, this will presumably bork.
    return (*pbset.begin());
}


int CpdKernel::execute()
{
    PointContext ctxX, ctxY;

    arma::mat X, Y;
    PointBufferPtr bufX = readFile(m_filex, ctxX, X);
    readFile(m_filey, ctxY, Y);

    if (X.n_rows == 0 || Y.n_rows == 0)
    {
        throw pdal_error("No points to process.");
    }

    cpd::NonrigidLowrank reg(m_tolerance,
                             m_max_it,
                             m_outliers,
                             m_fgt,
                             m_epsilon,
                             m_beta,
                             m_lambda,
                             m_numeig);
    reg.set_sigma2(m_sigma2);
    if (m_auto_z_exaggeration)
    {
        BOX3D bounds = bufX->calculateBounds();
        double min_range = std::min(bounds.maxx - bounds.minx, bounds.maxy - bounds.miny);
        double exaggeration = m_auto_z_exaggeration_ratio * min_range / (bounds.maxz - bounds.minz);
        reg.set_z_exaggeration(exaggeration);
    }

    cpd::Registration::ResultPtr result;
    PointContext ctxout;
    ctxout.registerDim(Dimension::Id::X);
    ctxout.registerDim(Dimension::Id::Y);
    ctxout.registerDim(Dimension::Id::Z);
    ctxout.registerDim(Dimension::Id::XVelocity);
    ctxout.registerDim(Dimension::Id::YVelocity);
    ctxout.registerDim(Dimension::Id::ZVelocity);
    PointBufferPtr bufout(new PointBuffer(ctxout));

    if (m_chipped)
    {
        result = chipThenRegister(reg, X, Y, bufX, ctxX);
        for (arma::uword i = 0; i < result->Y.n_rows; ++i)
        {
            bufout->setField<double>(Dimension::Id::X, i, result->Y(i, 0));
            bufout->setField<double>(Dimension::Id::Y, i, result->Y(i, 1));
            bufout->setField<double>(Dimension::Id::Z, i, result->Y(i, 2));
            bufout->setField<double>(Dimension::Id::XVelocity, i, result->Y(i, 3));
            bufout->setField<double>(Dimension::Id::YVelocity, i, result->Y(i, 4));
            bufout->setField<double>(Dimension::Id::ZVelocity, i, result->Y(i, 5));
        }
    }
    else
    {
        result = reg.run(X, Y);
        for (arma::uword i = 0; i < Y.n_rows; ++i)
        {
            bufout->setField<double>(Dimension::Id::X, i, result->Y(i, 0));
            bufout->setField<double>(Dimension::Id::Y, i, result->Y(i, 1));
            bufout->setField<double>(Dimension::Id::Z, i, result->Y(i, 2));
            bufout->setField<double>(Dimension::Id::XVelocity, i, Y(i, 0) - result->Y(i, 0));
            bufout->setField<double>(Dimension::Id::YVelocity, i, Y(i, 1) - result->Y(i, 1));
            bufout->setField<double>(Dimension::Id::ZVelocity, i, Y(i, 2) - result->Y(i, 2));
        }
    }

    BufferReader reader;
    reader.addBuffer(bufout);

    Options writerOpts;
    writerOpts.add<std::string>("filename", m_output);
    writerOpts.add<std::string>("order", "X,Y,Z,XVelocity,YVelocity,ZVelocity");
    writerOpts.add<bool>("keep_unspecified", false);
    setCommonOptions(writerOpts);

    std::unique_ptr<Stage> writer(KernelSupport::makeWriter(m_output, &reader));
    writer->setOptions(writerOpts + writer->getOptions());
    writer->prepare(ctxout);
    writer->execute(ctxout);

    return 0;
}


arma::mat getChip(const arma::mat& X, const BOX3D& bounds)
{
    std::vector<arma::uword> idx;
    for (arma::uword i = 0; i < X.n_rows; ++i)
    {
        if (bounds.contains(X(i, 0), X(i, 1), X(i, 2)))
        {
            idx.push_back(i);
        }
    }
    arma::uvec idxvec(idx);

    return X.rows(idxvec);
}


cpd::Registration::ResultPtr CpdKernel::chipThenRegister(const cpd::NonrigidLowrank& reg,
                                                        const arma::mat& X, const arma::mat& Y,
                                                        const PointBufferPtr& bufX,
                                                        const PointContext& ctx)
{
    BufferReader reader;
    reader.addBuffer(bufX);

    ChipperFilter chipper;
    chipper.setInput(&reader);
    Options options;
    options.add<int>("capacity", m_chip_capacity);
    chipper.setOptions(options);

    chipper.prepare(ctx);
    PointBufferSet pbSet = chipper.execute(ctx);
    std::cerr << "Number of chips: " << pbSet.size() << std::endl;

    cpd::Registration::ResultPtr result(new cpd::Registration::Result());
    int count = 0;
    for (auto it = pbSet.begin(); it != pbSet.end(); ++it)
    {
        BOX3D bounds = (*it)->calculateBounds();
        BOX3D chipBounds(bounds.minx - m_chip_buffer,
                         bounds.miny - m_chip_buffer,
                         bounds.minz,
                         bounds.maxx + m_chip_buffer,
                         bounds.maxy + m_chip_buffer,
                         bounds.maxz);

        arma::mat Xchip = getChip(X, chipBounds);
        arma::mat Ychip = getChip(Y, chipBounds);

        cpd::Registration::ResultPtr tmpresult = reg.run(Xchip, Ychip);
        result->Y.insert_rows(result->Y.n_rows, getChip(arma::join_horiz(tmpresult->Y, Ychip - tmpresult->Y), bounds));
        std::cerr << "Done with chip #" << ++count << " of " << pbSet.size() << std::endl;
    }

    return result;
}


}
