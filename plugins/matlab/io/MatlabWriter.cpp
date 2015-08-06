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


namespace pdal
{


static PluginInfo const s_info = PluginInfo(
    "writers.matlab",
    "Matlab .mat file writer.",
    "http://pdal.io/stages/writers.matlab.html");


CREATE_SHARED_PLUGIN(1, 0, MatlabWriter, Writer, s_info)
std::string MatlabWriter::getName() const { return s_info.name; }


MatlabWriter::MatlabWriter()
{}


void MatlabWriter::processOptions(const Options& options)
{}


void MatlabWriter::prepared(PointTableRef table)
{
    if (m_outputDims.empty())
    {
        m_dimTypes = table.layout()->dimTypes();
    }
    else
    {
        for (std::string& s : m_outputDims)
        {
            DimType dimType = table.layout()->findDimType(s);
            if (dimType.m_id == Dimension::Id::Unknown)
            {
                std::ostringstream oss;
                oss << "Invalid dimension '" << s << "' specified for "
                    "'output_dims' option.";
                throw pdal_error(oss.str());
            }
            m_dimTypes.push_back(dimType);
        }
    }
}


void MatlabWriter::ready(PointTableRef table)
{
    m_matfile = matOpen(m_filename.c_str(), "w");
    if (!m_matfile)
    {
        std::stringstream ss;
        ss << "Could not open file for writing: " << m_filename;
        throw pdal_error(ss.str());
    }
}


void MatlabWriter::write(const PointViewPtr view)
{
    point_count_t nPoints = view->size();
    auto nDimensions = m_dimTypes.size();

    std::stringstream dimensionsString;
    for (size_t i = 0; i < nDimensions; ++i)
    {
        if (i > 0) dimensionsString << ",";
        dimensionsString << Dimension::name(m_dimTypes[i].m_id);
    }
    mxArray * dimensionNames = mxCreateString(dimensionsString.str().c_str());
    if (!dimensionNames)
    {
        std::stringstream ss;
        ss << "Could not create string '" << dimensionsString.str() << "'";
        throw pdal_error(ss.str());
    }
    int result = matPutVariable(m_matfile, "Dimensions", dimensionNames);
    if (result != 0)
    {
        std::stringstream ss;
        ss << "Could not write dimension names to file: " << m_filename;
        throw pdal_error(ss.str());
    }

    mxArray * points = mxCreateDoubleMatrix(nPoints, nDimensions, mxREAL);
    if (!points) {
        std::stringstream ss;
        ss << "Could not create a points array with dimensions " << nPoints << "x" << nDimensions;
        throw pdal_error(ss.str());
    }

    double * pointsPtr = mxGetPr(points);
    // Matlab is column-major
    for (size_t j = 0; j < nDimensions; ++j)
    {
        for (point_count_t i = 0; i < nPoints; ++i)
        {
            double value = view->getFieldAs<double>(m_dimTypes[j].m_id, i);
            memcpy(static_cast<void*>(pointsPtr++),
                    static_cast<void*>(&value),
                    sizeof(double));
        }
    }
    result = matPutVariable(m_matfile, "Points", points);
    if (result != 0)
    {
        std::stringstream ss;
        ss << "Could not write points to file: " << m_filename;
        throw pdal_error(ss.str());
    }
    mxDestroyArray(points);
}


void MatlabWriter::done(PointTableRef table)
{
    int result = matClose(m_matfile);
    if (result != 0)
    {
        std::stringstream ss;
        ss << "Unsuccessful write: " << m_filename;
        throw pdal_error(ss.str());
    }
}


}
