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

void Hdf5Handler::setLog(pdal::LogPtr log) {
    m_logger = log;
}

DimInfo::DimInfo(
    const std::string& dimName,
    H5::IntType int_type)
    : name(dimName)
    , hdf_type(H5T_INTEGER)
    , endianness(int_type.getOrder())
    , sign(int_type.getSign())
    , size(int_type.getSize())
    , pdal_type(sign == H5T_SGN_2 ?
        Dimension::Type(unsigned(Dimension::BaseType::Signed) | int_type.getSize()) :
        Dimension::Type(unsigned(Dimension::BaseType::Unsigned) | int_type.getSize()))
    { }

DimInfo::DimInfo(
    const std::string& dimName,
    H5::FloatType float_type)
    : name(dimName)
    , hdf_type(H5T_FLOAT)
    , endianness(float_type.getOrder())
    , sign(H5T_SGN_ERROR)
    , size(float_type.getSize())
    , pdal_type(Dimension::Type(unsigned(Dimension::BaseType::Floating) | float_type.getSize()))
    { }

void Hdf5Handler::initialize(
        const std::string& filename,
        const NL::json& map)
{
    try
    {
        m_h5File.reset(new H5::H5File(filename, H5F_ACC_RDONLY));
    }
    catch (const H5::FileIException&)
    {
        throw pdal_error("Could not open HDF5 file '" + filename + "'.");
    }
    for(const auto& [dimName, datasetName] : map.items()) {
        m_logger->get(LogLevel::Warning) << "Opening dataset '"
            << datasetName << "' with dimension name '" << dimName
            << "'" << std::endl;
        H5::DataSet dset = m_h5File.get()->openDataSet(datasetName);
        m_dsets.push_back(dset);
        H5::DataSpace dspace = dset.getSpace();
        m_dspaces.push_back(dspace);
        m_numPoints = dspace.getSelectNpoints();
        H5::DSetCreatPropList plist = dset.getCreatePlist();
        if(plist.getLayout() == H5D_CHUNKED) {
            int dimensionality = plist.getChunk(1, &m_chunkSize);
            if(dimensionality != 1)
                throw pdal_error("Only 1-dimensional arrays are supported.");
        } else {
            m_logger->get(LogLevel::Warning) << "Dataset not chunked; proceeding to read 1024 elements at a time" << std::endl;
            m_chunkSize = 1024;
        }
        m_logger->get(LogLevel::Warning) << "Chunk size: " << m_chunkSize << std::endl;
        m_logger->get(LogLevel::Warning) << "Num points: " << m_numPoints << std::endl;
        H5::DataType dtype = dset.getDataType();
        H5T_class_t vague_type = dtype.getClass();

        if(vague_type == H5T_COMPOUND) {
            throw pdal_error("Compound types not supported");
        }
        else if(vague_type == H5T_INTEGER) {
            m_dimInfos.push_back(
                DimInfo(dimName, dset.getIntType())
            );
        }
        else if(vague_type == H5T_FLOAT) {
            m_dimInfos.push_back(
                DimInfo(dimName, dset.getFloatType())
            );
        } else {
            throw pdal_error("Unkown type: " + vague_type);
        }
        m_data.resize(m_chunkSize*dtype.getSize());
    }
}

void Hdf5Handler::close()
{
    m_h5File->close();
}


uint8_t *Hdf5Handler::getNextChunk() {
    hsize_t elementsRemaining = m_numPoints - m_chunkOffset;
    hsize_t selectionSize = std::min(elementsRemaining, m_chunkSize);

    H5::DataSpace memspace(1, &selectionSize);
    m_dspaces.at(0).selectHyperslab(H5S_SELECT_SET, &selectionSize, &m_chunkOffset);
    m_dsets.at(0).read(m_data.data(),
                m_dsets.at(0).getDataType(),
                memspace,
                m_dspaces.at(0) );
    m_chunkOffset += m_chunkSize;
    return m_data.data();
}


uint64_t Hdf5Handler::getNumPoints() const
{
    return m_numPoints;
}


std::vector<pdal::hdf5::DimInfo> Hdf5Handler::getDimensionInfos() {
    return m_dimInfos;
}


hsize_t Hdf5Handler::getChunkSize() {
    return m_chunkSize;
}

} // namespace pdal

