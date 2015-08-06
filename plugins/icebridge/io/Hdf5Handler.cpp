/******************************************************************************
* Copyright (c) 2014, Connor Manning, connor@hobu.co
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

#include "Hdf5Handler.hpp"
#include <pdal/util/FileUtils.hpp>
#include <pdal/pdal_types.hpp>

#include <boost/numeric/conversion/cast.hpp>


namespace pdal
{

using namespace hdf5;

Hdf5Handler::Hdf5Handler()
    : m_numPoints(0)
    , m_columnDataMap()
{ }

void Hdf5Handler::initialize(
        const std::string& filename,
        const std::vector<Hdf5ColumnData>& columns)
{
    try
    {
        m_h5File.reset(new H5::H5File(filename, H5F_ACC_RDONLY));
    }
    catch (const H5::FileIException&)
    {
        throw pdal_error("Could not open HDF5 file.");
    }

    try
    {
        // Open each HDF5 DataSet and its corresponding DataSpace.
        for (const auto& col : columns)
        {
            const std::string dataSetName = col.name;
            const H5::PredType predType = col.predType;
            const H5::DataSet dataSet = m_h5File->openDataSet(dataSetName);
            const H5::DataSpace dataSpace = dataSet.getSpace();

            m_columnDataMap.insert(std::make_pair(
                        dataSetName,
                        ColumnData(predType, dataSet, dataSpace)));

            // Does not check whether all the columns are the same length.
            m_numPoints =
                std::max((uint64_t)getColumnNumEntries(dataSetName), m_numPoints);
        }
    }
    catch (const H5::Exception&)
    {
        throw pdal_error("Could not initialize data set information.");
    }
}

void Hdf5Handler::close()
{
    m_h5File->close();
}

uint64_t Hdf5Handler::getNumPoints() const
{
    return m_numPoints;
}

void Hdf5Handler::getColumnEntries(
        void* data,
        const std::string& dataSetName,
        const hsize_t numEntries,
        const hsize_t offset) const
{
    try
    {
        const ColumnData& columnData(getColumnData(dataSetName));

        columnData.dataSpace.selectHyperslab(
                H5S_SELECT_SET,
                &numEntries,
                &offset);

        const hsize_t outOffset = 0;
        const H5::DataSpace outSpace(1, &numEntries);
        outSpace.selectHyperslab(H5S_SELECT_SET, &numEntries, &outOffset);

        columnData.dataSet.read(
                data,
                columnData.predType,
                outSpace, 
                columnData.dataSpace);
    }
    catch (const H5::Exception&)
    {
        throw pdal_error("Could not read from dataset.");
    }
}

hsize_t 
Hdf5Handler::getColumnNumEntries(const std::string& dataSetName) const
{
    hsize_t entries = 0;

    getColumnData(dataSetName).dataSpace.getSimpleExtentDims(&entries);

    return entries;
}

const Hdf5Handler::ColumnData&
Hdf5Handler::getColumnData(const std::string& dataSetName) const
{
    const auto columnDataIt(m_columnDataMap.find(dataSetName));

    if (columnDataIt == m_columnDataMap.end())
    {
        throw pdal_error("Could not retrieve column data.");
    }

    return columnDataIt->second;
}

} // namespace pdal

