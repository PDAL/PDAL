/******************************************************************************
 * Copyright (c) 2015, Peter J. Gadomski <pete.gadomski@gmail.com>
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

#include "MatlabWriter.hpp"
#include "../filters/Script.hpp"

#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "writers.matlab",
    "Matlab .mat file writer.",
    "http://pdal.io/stages/writers.matlab.html"
};

CREATE_SHARED_STAGE(MatlabWriter, s_info)
std::string MatlabWriter::getName() const { return s_info.name; }

void MatlabWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("output_dims", "Output dimensions", m_outputDims);
    args.add("struct", "Matlab struct name", m_structName, "PDAL");
}


void MatlabWriter::prepared(PointTableRef table)
{
    if (m_outputDims.empty())
    {
        m_dims = table.layout()->dims();
    }
    else
    {
        for (std::string& s : m_outputDims)
        {
//             DimType dimType = table.layout()->findDimType(s);
            Dimension::Id id = table.layout()->findDim(s);
            if (id == Dimension::Id::Unknown)
                throwError("Invalid dimension '" + s + "' specified for "
                    "'output_dims' option.");
            m_dims.push_back(id);
        }
    }
}


void MatlabWriter::ready(PointTableRef table)
{
    m_matfile = matOpen(m_filename.c_str(), "w");
    if (!m_matfile)
        throwError("Could not open file '" + m_filename + "' for writing.");

    m_tableMetadata = table.metadata();
}


void MatlabWriter::write(const PointViewPtr view)
{

    mxArray* data = mlang::Script::setMatlabStruct(view, m_dims, "", m_tableMetadata, log());
    if (matPutVariable(m_matfile, m_structName.c_str(), data))
        throwError("Could not write points to file '" + m_filename + "'.");

    mxDestroyArray(data);
}


void MatlabWriter::done(PointTableRef table)
{
    if (matClose(m_matfile))
        throwError("Unsuccessful write.");
    getMetadata().addList("filename", m_filename);
}

}
