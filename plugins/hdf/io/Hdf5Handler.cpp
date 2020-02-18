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
#include <pdal/Dimension.hpp>

namespace pdal
{

using namespace hdf5;

void Handler::setLog(pdal::LogPtr log) {
    m_logger = log;
}

DimInfo::DimInfo(
    const std::string& dimName,
    const std::string& datasetName,
    std::shared_ptr<H5::H5File> file
    )
    : name(dimName)
    , hdfPath(datasetName)
    , chunkSize(1024)
    {
        // Will throw if dataset doesn't exists. Gives adequate error message
        H5::DataSet dset = file.get()->openDataSet(datasetName);
        H5::DataSpace dspace = dset.getSpace();
        if(dspace.getSelectNpoints() < 0)
            throw pdal_error("Selection had a negative number of points. "
                "this should never happen, and it's probably a PDAL bug.");
        
        numPoints = (hsize_t) dspace.getSelectNpoints();
        H5::DSetCreatPropList plist = dset.getCreatePlist();
        if(plist.getLayout() == H5D_CHUNKED) {
            int dimensionality = plist.getChunk(1, &chunkSize); //modifies chunkSize
            if(dimensionality != 1)
                throw pdal_error("Only 1-dimensional arrays are supported.");
        } else {
            chunkSize = 1024;
        }
        H5T_class_t vague_type = dset.getDataType().getClass();

        if(vague_type == H5T_INTEGER) {
            auto int_type = dset.getIntType();
            sign = int_type.getSign();
            size = int_type.getSize();
            if(sign == H5T_SGN_2)
                pdal_type = Dimension::Type(unsigned(Dimension::BaseType::Signed) | int_type.getSize());
            else
                pdal_type = Dimension::Type(unsigned(Dimension::BaseType::Unsigned) | int_type.getSize());
        }
        else if(vague_type == H5T_FLOAT) {
            auto float_type = dset.getFloatType();
            sign = H5T_SGN_ERROR;
            size = float_type.getSize();
            pdal_type = Dimension::Type(unsigned(Dimension::BaseType::Floating) | float_type.getSize());
        }
        else {
            throw pdal_error("Dataset '" + datasetName + "' has an " +
                "unsupported type. Only integer and float types are supported.");
        }

        buffer.resize(chunkSize*size);
    }

void Handler::initialize(
        const std::string& filename,
        const std::map<std::string,std::string>& map)
{
    try
    {
        m_h5File.reset(new H5::H5File(filename, H5F_ACC_RDONLY));
    }
    catch (const H5::FileIException&)
    {
        throw pdal_error("Could not open HDF5 file '" + filename + "'.");
    }

    for( auto const& entry : map) {
        std::string const& dimName = entry.first;
        std::string const& datasetName = entry.second;
        m_dimInfos.emplace_back(DimInfo(dimName, datasetName, m_h5File));
    }

    m_numPoints = m_dimInfos.at(0).numPoints;
    for( auto& info : m_dimInfos) {
        if(m_numPoints != info.numPoints) {
            throw pdal_error("All given datasets must have the same length");
        }
    }
}

void Handler::close()
{
    m_h5File->close();
}


uint8_t *Handler::getValue(DimInfo& info, pdal::point_count_t pointIndex) {
    uint8_t *p = info.buffer.data();

    if(pointIndex < info.chunkLowerBound || pointIndex >= info.chunkUpperBound) {
        // load new chunk
        auto dset = m_h5File.get()->openDataSet(info.hdfPath);
        auto dspace = dset.getSpace();

        info.chunkLowerBound = (pointIndex / info.chunkSize) * info.chunkSize;
        info.chunkUpperBound = std::min(info.chunkLowerBound + info.chunkSize, m_numPoints);

        hsize_t selectionSize = info.chunkUpperBound - info.chunkLowerBound;

        H5::DataSpace memspace(1, &selectionSize);
        dspace.selectHyperslab(H5S_SELECT_SET, &selectionSize, &info.chunkLowerBound);
        dset.read(  p,
                    dset.getDataType(),
                    memspace,
                    dspace );

    }
    hsize_t pointOffsetWithinChunk = pointIndex - info.chunkLowerBound;
    return p + pointOffsetWithinChunk * info.size;
}


hsize_t Handler::getNumPoints() const
{
    return m_numPoints;
}


std::vector<pdal::hdf5::DimInfo>& Handler::getDimensionInfos() {
    return m_dimInfos;
}


} // namespace pdal

