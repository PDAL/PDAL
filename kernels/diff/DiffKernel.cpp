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

#include "DiffKernel.hpp"

#include <memory>

#include <pdal/PDALUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/pdal_macros.hpp>


namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.diff",
    "Diff Kernel",
    "http://pdal.io/kernels/kernels.diff.html" );

CREATE_STATIC_PLUGIN(1, 0, DiffKernel, Kernel, s_info)

std::string DiffKernel::getName() const { return s_info.name; }

void DiffKernel::addSwitches(ProgramArgs& args)
{
    Arg& source = args.add("source", "Source filename", m_sourceFile);
    source.setPositional();
    Arg& candidate = args.add("candidate", "Candidate filename",
        m_candidateFile);
    candidate.setPositional();
}


void DiffKernel::checkPoints(const PointView& source_data,
    const PointView& candidate_data, MetadataNode errors)
{
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
            Dimension::Id sd = sourceDims[d];
            Dimension::Id cd = candidateDims[d];

            source_data.getRawField(sd, idx, (void *)sbuf);
            candidate_data.getRawField(cd, idx, (void *)cbuf);
            Dimension::Type t = Dimension::defaultType(cd);
            size_t size = Dimension::size(t);
            if (memcmp(sbuf, cbuf, size))
            {
                std::ostringstream oss;

                oss << "Point " << idx << " differs for dimension \"" <<
                    Dimension::name(sd) << "\" for source and candidate";
                errors.add("data.error", oss.str());
                badbytes++;
            }
        }
        if (badbytes > MAX_BADBYTES )
            break;
    }
}


int DiffKernel::execute()
{
    PointTable sourceTable;

    Stage& source = makeReader(m_sourceFile, m_driverOverride);
    source.prepare(sourceTable);
    PointViewSet sourceSet = source.execute(sourceTable);

    MetadataNode errors;

    PointTable candidateTable;

    Stage& candidate = makeReader(m_candidateFile, m_driverOverride);
    candidate.prepare(candidateTable);
    PointViewSet candidateSet = candidate.execute(candidateTable);

    assert(sourceSet.size() == 1);
    assert(candidateSet.size() == 1);
    PointViewPtr sourceView = *sourceSet.begin();
    PointViewPtr candidateView = *candidateSet.begin();
    if (candidateView->size() != sourceView->size())
    {
        std::ostringstream oss;

        oss << "Source and candidate files do not have the same point count";
        errors.add("count.error", oss.str());
        errors.add("count.candidate", candidateView->size());
        errors.add("count.source", sourceView->size());
    }

    MetadataNode source_metadata = sourceTable.metadata();
    MetadataNode candidate_metadata = candidateTable.metadata();
    if (source_metadata != candidate_metadata)
    {
        std::ostringstream oss;

        oss << "Source and candidate files do not have the same metadata count";
        errors.add("metadata.error", oss.str());
        errors.add(source_metadata);
        errors.add(candidate_metadata);
    }

    if (candidateTable.layout()->dims().size() !=
        sourceTable.layout()->dims().size())
    {
        std::ostringstream oss;

        oss << "Source and candidate files do not have the same "
            "number of dimensions";
//         errors.put<std::string>("schema.error", oss.str());
        //Need to "ptree" the PointTable dimension list in some way
        // errors.put_child("schema.source", sourceTable.schema()->toPTree());
        // errors.put_child("schema.candidate",
        //     candidateTable.schema()->toPTree());
    }

//     if (errors.size())
//     {
//         write_json(std::cout, errors);
//         return 1;
//     }
//     else
    {
        // If we made it this far with no errors, now we'll
        // check the points.
//         checkPoints(*sourceView, *candidateView, errors);
//         if (errors.size())
//         {
//             write_json(std::cout, errors);
//             return 1;
//         }
    }
    return 0;
}

} // namespace pdal

