/******************************************************************************
* Copyright (c) 2017, Howard Butler (howard@hobu.co)
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

#include "MatlabFilter.hpp"
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/FileUtils.hpp>

#include <algorithm>
#include <iterator>

namespace pdal
{

static PluginInfo const s_info
{
    "filters.matlab",
    "Manipulate data using inline Matlab",
    "http://pdal.io/stages/filters.matlab.html"
};

CREATE_SHARED_STAGE(MatlabFilter, s_info)

std::string MatlabFilter::getName() const { return s_info.name; }

void MatlabFilter::addArgs(ProgramArgs& args)
{
    args.add("source", "Matlab script to run", m_script.m_source);
    args.add("add_dimension", "Dimensions to add", m_addDimensions);
    args.add("struct", "Matlab struct array to read", m_structName, "PDAL");
    args.add("pdalargs", "Dictionary to add to module globals when calling function", m_pdalargs);
}


void MatlabFilter::addDimensions(PointLayoutPtr layout)
{
    for (const std::string& s : m_addDimensions)
        layout->registerOrAssignDim(s, pdal::Dimension::Type::Double);
}


void MatlabFilter::ready(PointTableRef table)
{
    if (m_script.m_source.empty())
        m_script.m_source = FileUtils::readFileIntoString(m_script.m_scriptFilename);

    m_tableMetadata = table.metadata();
}


PointViewSet MatlabFilter::run(PointViewPtr view)
{
    log()->get(LogLevel::Debug) << "filters.matlab " << m_script <<
        " processing " << view->size() << " points." << std::endl;

    int logBufferSize(4096);
    std::unique_ptr<char[]> buf(new char[logBufferSize]);
    m_MatlabOutputBuffer.swap(buf);

    Engine* engine = mlang::Environment::get()->m_engine;
    engOutputBuffer(engine, m_MatlabOutputBuffer.get(), logBufferSize);

    Dimension::IdList dims;

    mxArray* matlabData = mlang::Script::setMatlabStruct(view, dims, m_pdalargs, m_tableMetadata, log());
    if (engPutVariable(engine, m_structName.c_str(), matlabData))
    {
        std::ostringstream oss;
        oss << "Could not push '" << m_structName << "' struct to Matlab";
        throwError(oss.str());
    }

    engEvalString(engine, m_script.m_source.c_str());

    std::string noise(m_MatlabOutputBuffer.get(), strlen(m_MatlabOutputBuffer.get()));
    log()->get(LogLevel::Debug) << "filters.matlab " << noise << std::endl;

    matlabData = engGetVariable(engine, m_structName.c_str());
    if (!matlabData)
        throwError("No 'PDAL' variable is available in Matlab scope!");

    PointViewSet viewSet;

    std::string logicalDimensionName = m_script.getLogicalMask(matlabData, log());
    if (logicalDimensionName.size())
    {
        PointViewPtr outview = view->makeNew();

        mxArray* f = mxGetField(matlabData, 0, logicalDimensionName.c_str());
        if (!f)
        {
            std::ostringstream oss;
            oss << "Unable to fetch mask dimension '" << logicalDimensionName << "'";
            throwError(oss.str());
        }

        mxLogical* logical = mxGetLogicals(f);
        if (!logical)
        {
            std::ostringstream oss;
            oss << "Unable to fetch logical mask for dimension '" << logicalDimensionName << "'";
            throwError(oss.str());
        }

        char *ok = (char *)logical;
        for (PointId idx = 0; idx < view->size(); ++idx)
            if (*ok++)
                outview->appendPoint(*view, idx);
        viewSet.insert(outview);
    }
    else
    {
        mlang::Script::getMatlabStruct(matlabData, view, dims, m_pdalargs, m_tableMetadata, log());
        viewSet.insert(view);
    }
    return viewSet;

}


void MatlabFilter::done(PointTableRef table)
{
}

} // namespace pdal
