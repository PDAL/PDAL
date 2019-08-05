/******************************************************************************
* Copyright (c) 2017, Hobu Inc., info@hobu.co
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
*     * Neither the name of Hobu, Inc. nor the
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

#include <pdal/PDALUtils.hpp>
#include <pdal/util/Algorithm.hpp>

#include "MatlabReader.hpp"

#include <pdal/util/Algorithm.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "readers.matlab",
    "Matlab Reader",
    "http://pdal.io/stages/readers.matlab.html"
};

CREATE_SHARED_STAGE(MatlabReader, s_info)
std::string MatlabReader::getName() const { return s_info.name; }

void MatlabReader::initialize(PointTableRef table)
{
    m_matfile = matOpen(m_filename.c_str(), "r");
    if (!m_matfile)
        throwError("Could not open file '" + m_filename + "' for reading.");

    m_structArray = matGetVariable(m_matfile, m_structName.c_str());
    if (!m_structArray)
    {
        std::ostringstream oss;
        oss << "Array struct with name '" << m_structName << "' not found ";
        oss << "in file '" << m_filename << "'";
        throwError(oss.str());
    }

    m_pointIndex = 0;
    m_numElements = 0;
    m_numFields = 0;

    setSpatialReference(mlang::Script::getSRSWKT(m_structArray, log()));
    m_tableMetadata = table.metadata();
}


void MatlabReader::addArgs(ProgramArgs& args)
{
    args.add("struct", "Name of struct to read from file",
        m_structName, "PDAL");
}


void MatlabReader::addDimensions(PointLayoutPtr layout)
{
    log()->get(LogLevel::Debug) << "Opening file" <<
        " '" << m_filename << "'." << std::endl;



    // For now we read all of the fields in the given
    // array structure
    m_numFields = mxGetNumberOfFields(m_structArray);
    if (!m_numFields)
        throw pdal::pdal_error("Selected struct array must have fields!");

    // Fetch the first array and determine number of elements
    // it has. We're only going to read that many elements no
    // matter what
    mxArray* f = mxGetFieldByNumber(m_structArray, 0, 0);
    if (!f)
    {
        throwError("Unable to fetch first array in array struct to determine number of elements!");
    }
    m_numElements = mxGetNumberOfElements(f);

    // Get the dimensions and their types from the array struct
    PointLayoutPtr matlabLayout = mlang::Script::getStructLayout(m_structArray, log());
    const Dimension::IdList& dims = matlabLayout->dims();

    int nDimensionNumber(0);
    for(auto d: dims)
    {
        std::string dimName = matlabLayout->dimName(d);
        const Dimension::Detail* detail = matlabLayout->dimDetail(d);
        Dimension::Id id = detail->id();
        Dimension::Type t = detail->type();
        layout->registerDim(id, t);

        // Keep a map of the Matlab dimension number to
        // both the PDAL type and PDAL id
        std::pair<int, int> pd = std::make_pair(nDimensionNumber, (int)id);
        m_dimensionIdMap.insert(pd);
        std::pair<int, int> pt = std::make_pair(nDimensionNumber, (int)t);
        m_dimensionTypeMap.insert(pt);
        nDimensionNumber++;
    }

}


point_count_t MatlabReader::read(PointViewPtr view, point_count_t numPts)
{
    PointId idx = view->size();
    point_count_t cnt = 0;
    PointRef point(*view, idx);
    while (cnt < numPts)
    {
        point.setPointId(idx);
        if (!processOne(point))
            break;
        cnt++;
        idx++;
    }
    return cnt;
}


bool MatlabReader::processOne(PointRef& point)
{
    // We read them all
    if (m_pointIndex == m_numElements)
        return false;

    for (int i=0; i < m_numFields; ++i)
    {
        Dimension::Id d = (Dimension::Id) m_dimensionIdMap[i];
        Dimension::Type t = (Dimension::Type) m_dimensionTypeMap[i];

        mxArray* f = mxGetFieldByNumber(m_structArray, 0, i);
        if (!f)
        {
            std::ostringstream oss;
            oss << "Unable to fetch array for point " << m_pointIndex;
            throwError(oss.str());
        }
        PointId numElements = (PointId) mxGetNumberOfElements(f);
        if (numElements >= m_pointIndex )
        {
            size_t size = mxGetElementSize(f);
            char* p = (char*)mxGetData(f) + (m_pointIndex*size);
            point.setField(d, t, (void*)p);
        }
    }

    m_pointIndex++;

    return true;
}


void MatlabReader::done(PointTableRef table)
{
    matClose(m_matfile);
    getMetadata().addList("filename", m_filename);
    getMetadata().add("struct", m_structName);
}


} // namespace pdal

