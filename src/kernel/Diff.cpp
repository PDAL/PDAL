/******************************************************************************
* Copyright (c) 2014, Howard Butler (howard@hobu.co)
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

#include <pdal/kernel/Diff.hpp>
#include <pdal/PDALUtils.hpp>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <boost/scoped_ptr.hpp>

using boost::property_tree::ptree;

namespace pdal {
namespace kernel {

Diff::Diff(int argc, const char* argv[])
    : Application(argc, argv, "dif")
    , m_sourceFile("")
    , m_candidateFile("")
    , m_useXML(false)
    , m_useJSON(false)

{}


void Diff::validateSwitches()
{
    if (!m_sourceFile.size())
        throw app_runtime_error("No source file given!");
    if (!m_candidateFile.size())
        throw app_runtime_error("No candidate file given!");
}


void Diff::addSwitches()
{
    namespace po = boost::program_options;

    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
        ("source", po::value<std::string>(&m_sourceFile), "source file name")
        ("candidate",
            po::value<std::string>(&m_candidateFile), "candidate file name")
        ("xml",
            po::value<bool>(&m_useXML)->zero_tokens()->implicit_value(true),
            "dump XML")
        ("json",
            po::value<bool>(&m_useJSON)->zero_tokens()->implicit_value(true),
            "dump JSON")
    ;

    addSwitchSet(file_options);
    po::options_description* processing_options =
        new po::options_description("processing options");

    processing_options->add_options();

    addSwitchSet(processing_options);

    addPositionalSwitch("source", 1);
    addPositionalSwitch("candidate", 2);
}


void Diff::checkPoints(const PointBuffer& source_data,
    const PointBuffer& candidate_data, ptree& errors)
{
    uint32_t i(0);
    uint32_t MAX_BADBYTES(20);
    uint32_t badbytes(0);

    // Both schemas have already been determined to be equal, so are the
    // same size and in the same order.
    Dimension::IdList const& sourceDims = source_data.dims();
    Dimension::IdList const& candidateDims = candidate_data.dims();

    char sbuf[8];
    char cbuf[8];
    for (PointId idx = 0; idx < source_data.size(); ++idx)
    {
        for (size_t d = 0; d < sourceDims.size(); ++d)
        {
            Dimension::Id::Enum sd = sourceDims[d];
            Dimension::Id::Enum cd = candidateDims[d];

            source_data.getRawField(sd, idx, (void *)sbuf);
            candidate_data.getRawField(cd, idx, (void *)cbuf);
            Dimension::Type::Enum t = Dimension::defaultType(cd);
            size_t size = Dimension::size(t);
            if (memcmp(sbuf, cbuf, size))
            {
                std::ostringstream oss;

                oss << "Point " << idx << " differs for dimension \"" <<
                    Dimension::name(sd) << "\" for source and candidate";
                errors.put<std::string>("data.error", oss.str());
                badbytes++;
            }
        }
        if (badbytes > MAX_BADBYTES )
            break;
    }
}


int Diff::execute()
{
    PointContext sourceCtx;

    Options sourceOptions;
    {
        sourceOptions.add<std::string>("filename", m_sourceFile);
        sourceOptions.add<bool>("debug", isDebug());
        sourceOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    }
    std::unique_ptr<Stage> source(AppSupport::makeReader(sourceOptions));
    source->prepare(sourceCtx);
    PointBufferSet sourceSet = source->execute(sourceCtx);

    ptree errors;

    PointContext candidateCtx;
    Options candidateOptions;
    {
        candidateOptions.add<std::string>("filename", m_candidateFile);
        candidateOptions.add<bool>("debug", isDebug());
        candidateOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    }

    std::unique_ptr<Stage> candidate(AppSupport::makeReader(candidateOptions));
    candidate->prepare(candidateCtx);
    PointBufferSet candidateSet = candidate->execute(candidateCtx);

    assert(sourceSet.size() == 1);
    assert(candidateSet.size() == 1);
    PointBufferPtr sourceBuf = *sourceSet.begin();
    PointBufferPtr candidateBuf = *candidateSet.begin();
    if (candidateBuf->size() != sourceBuf->size())
    {
        std::ostringstream oss;

        oss << "Source and candidate files do not have the same point count";
        errors.put("count.error", oss.str());
        errors.put("count.candidate", candidateBuf->size());
        errors.put("count.source", sourceBuf->size());
    }

    MetadataNode source_metadata = sourceCtx.metadata();
    MetadataNode candidate_metadata = candidateCtx.metadata();
    if (source_metadata != candidate_metadata)
    {
        std::ostringstream oss;

        oss << "Source and candidate files do not have the same metadata count";
        errors.put("metadata.error", oss.str());
        errors.put_child("metadata.source", pdal::utils::toPTree(source_metadata));
        errors.put_child("metadata.candidate", pdal::utils::toPTree(candidate_metadata));
    }

    if (candidateCtx.dims().size() != sourceCtx.dims().size())
    {
        std::ostringstream oss;

        oss << "Source and candidate files do not have the same "
            "number of dimensions";
        errors.put<std::string>("schema.error", oss.str());
        //Need to "ptree" the PointContext dimension list in some way
        // errors.put_child("schema.source", sourceCtx.schema()->toPTree());
        // errors.put_child("schema.candidate",
        //     candidateCtx.schema()->toPTree());
    }

    if (errors.size())
    {
        write_json(std::cout, errors);
        return 1;
    }
    else
    {
        // If we made it this far with no errors, now we'll
        // check the points.
        checkPoints(*sourceBuf, *candidateBuf, errors);
        if (errors.size())
        {
            write_json(std::cout, errors);
            return 1;
        }
    }
    return 0;
}

} // namespace kernel
} // namespace pdal
