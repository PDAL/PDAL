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

    // Create our vector of dimensions and associated data
    for( auto const& entry : map) {
        std::string const& dimName = entry.first;
        std::string const& datasetName = entry.second;
        m_dimInfos.emplace_back(DimInfo(dimName, datasetName, m_h5File.get()));
    }

    // Check that all dimensions have equal lengths
    m_numPoints = m_dimInfos.at(0).getNumPoints();
    for( DimInfo& info : m_dimInfos) {
        if(m_numPoints != info.getNumPoints()) {
            throw pdal_error("All given datasets must have the same length");
        }
    }
}


void Handler::close()
{
    m_h5File->close();
}


uint8_t *DimInfo::getValue(pdal::point_count_t pointIndex) {
    if(pointIndex < chunkLowerBound || pointIndex >= chunkUpperBound) {
        // load new chunk
        H5::DataSpace dspace = m_dset.getSpace();

        chunkLowerBound = (pointIndex / m_chunkSize) * m_chunkSize;
        chunkUpperBound =
            (std::min)(chunkLowerBound + m_chunkSize, m_numPoints);

        hsize_t selectionSize = chunkUpperBound - chunkLowerBound;

        H5::DataSpace memspace(1, &selectionSize);
        dspace.selectHyperslab(H5S_SELECT_SET, &selectionSize, &chunkLowerBound);
        m_dset.read(m_buffer.data(),
                    m_dset.getDataType(),
                    memspace,
                    dspace );

    }
    hsize_t pointOffsetWithinChunk = pointIndex - chunkLowerBound;
    return m_buffer.data() + pointOffsetWithinChunk * m_size;
}


hsize_t Handler::getNumPoints() const
{
    return m_numPoints;
}

DimInfo::DimInfo(
    const std::string& dimName,
    const std::string& datasetName,
    H5::H5File *file
    )
    : m_name(dimName)
    , m_dset(file->openDataSet(datasetName))
    {
        // Will throw if dataset doesn't exists. Gives adequate error message
        H5::DataSpace dspace = m_dset.getSpace();

        // Sanity check before we cast from signed to unsigned
        if(dspace.getSelectNpoints() < 0)
            throw pdal_error("Selection had a negative number of points. "
                "this should never happen, and it's probably a PDAL bug.");     
        m_numPoints = (hsize_t) dspace.getSelectNpoints();

        // check if dataset is 'chunked'
        H5::DSetCreatPropList plist = m_dset.getCreatePlist();
        if(plist.getLayout() == H5D_CHUNKED) {
            int dimensionality = plist.getChunk(1, &m_chunkSize); //modifies m_chunkSize
            if(dimensionality != 1)
                throw pdal_error("Only 1-dimensional arrays are supported.");
        } else {
            //if dataset is not chunked, use an arbitrary number for buffer size
            m_chunkSize = 1024; // completely arbitrary number
        }

        // populate fields base on HDF type
        H5T_class_t vague_type = m_dset.getDataType().getClass();

        if(vague_type == H5T_INTEGER) {
            H5::IntType int_type = m_dset.getIntType();
            H5T_sign_t sign = int_type.getSign();
            m_size = int_type.getSize();
            if(sign == H5T_SGN_2)
                m_pdalType = Dimension::Type(unsigned(Dimension::BaseType::Signed) | int_type.getSize());
            else
                m_pdalType = Dimension::Type(unsigned(Dimension::BaseType::Unsigned) | int_type.getSize());
        }
        else if(vague_type == H5T_FLOAT) {
            H5::FloatType float_type = m_dset.getFloatType();
            m_size = float_type.getSize();
            m_pdalType = Dimension::Type(unsigned(Dimension::BaseType::Floating) | float_type.getSize());
        }
        else {
            throw pdal_error("Dataset '" + datasetName + "' has an " +
                "unsupported type. Only integer and float types are supported.");
        }

        //allocate buffer for getValue() to write into
        m_buffer.resize(m_chunkSize*m_size);
    }


std::vector<pdal::hdf5::DimInfo>& Handler::getDimensions() {
    return m_dimInfos;
}


void DimInfo::setId(Dimension::Id id) {
    m_pdalId = id;
}


Dimension::Id DimInfo::getId() {
    return m_pdalId;
}


Dimension::Type DimInfo::getPdalType() {
    return m_pdalType;
}


std::string DimInfo::getName() {
    return m_name;
}


hsize_t DimInfo::getNumPoints() {
    return m_numPoints;
}

} // namespace pdal

