/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
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

#include <pdal/filters/Index.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Utils.hpp>

#include <boost/concept_check.hpp> // ignore_unused_variable_warning

#include <algorithm>
#include <cmath>



namespace pdal
{
namespace filters
{


Index::Index(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
    , m_dimensions(3)
{
    return;
}

void Index::initialize()
{
    Filter::initialize();
    setNumDimensions(getOptions().getValueOrDefault<boost::uint32_t>("dimensions", 3));
    return;
}

boost::uint32_t const& Index::getNumDimensions() const
{
    return m_dimensions;
}

Options Index::getDefaultOptions()
{
    Options options;

    Option x("x_dim", std::string("X"), "Dimension name to use for 'X' data");
    Option y("y_dim", std::string("Y"), "Dimension name to use for 'Y' data");
    Option z("z_dim", std::string("Z"), "Dimension name to use for 'Z' data");
    Option no_dims("dimensions", 3, "Number of dimensions to use for index");

    options.add(x);
    options.add(y);
    options.add(z);
    options.add(no_dims);

    return options;
}

void Index::processBuffer(PointBuffer& /* data */) const
{
    return;
}


pdal::StageSequentialIterator* Index::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Index(*this, buffer);
}


namespace iterators
{
namespace sequential
{


Index::Index(const pdal::filters::Index& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_stage(filter)
    , m_numIndexPoints(0)
#ifdef PDAL_HAVE_FLANN
    , m_index(0)
    , m_dataset(0)
#endif
    , m_xDim(0)
    , m_yDim(0)
    , m_zDim(0)
{

    return;
}


void Index::readBufferBeginImpl(PointBuffer& buffer)
{
    // Cache dimension positions

    if (!m_xDim || !m_yDim || !m_zDim)
    {
        pdal::Schema const& schema = buffer.getSchema();

        std::string x_name = m_stage.getOptions().getValueOrDefault<std::string>("x_dim", "X");
        std::string y_name = m_stage.getOptions().getValueOrDefault<std::string>("y_dim", "Y");
        std::string z_name = m_stage.getOptions().getValueOrDefault<std::string>("z_dim", "Z");

        m_stage.log()->get(logDEBUG2) << "Indexing PointBuffer with X: '" << x_name
                                      << "' Y: '" << y_name
                                      << "' Z: '" << z_name << " with " <<  m_stage.getNumDimensions() << " dimensions" << std::endl;
        m_xDim = &schema.getDimension(x_name);
        m_yDim = &schema.getDimension(y_name);
        m_zDim = &schema.getDimension(z_name);


        if (!m_stage.getNumPoints())
            throw pdal_error("Unable to create index from pipeline that has an indeterminate number of points!");
    }

}

std::vector<boost::uint32_t> Index::query(double const& x, double const& y, double const& z, double distance, boost::uint32_t k)
{
    std::vector<boost::uint32_t> output;

#ifdef PDAL_HAVE_FLANN

    if (!m_index)
    {
        throw pdal_error("Index is not initialized! Unable to query!");
    }
    m_stage.log()->get(logDEBUG2)   << "Searching for x: "
                                    << x << " y: " << y << " z: " << z
                                    << " with distance threshold " << distance << std::endl;


    boost::uint32_t num_dimensions(m_stage.getNumDimensions());
    std::vector<float> distances_vec;
    distances_vec.resize(k);

    std::vector<boost::int32_t> indices_vec;
    indices_vec.resize(k);
    indices_vec.assign(indices_vec.size(), -1);

    std::vector<float> query_vec(num_dimensions);
    query_vec[0] = x;
    query_vec[1] = y;
    if (num_dimensions > 2)
        query_vec[2] = z;


    flann::Matrix<int> indices_mat(&indices_vec[0], 1, k);
    flann::Matrix<float> distances_mat(&distances_vec[0], 1, k);
    flann::Matrix<float> query_mat(&query_vec[0], 1, num_dimensions);

    m_index->knnSearch(query_mat,
                       indices_mat,
                       distances_mat,
                       k,
                       flann::SearchParams(128));

    bool logOutput = m_stage.log()->getLevel() > logDEBUG4;
    m_stage.log()->floatPrecision(8);

    for (unsigned i=0; i < k; ++i)
    {
        // if distance is 0, just return the nearest one, otherwise filter by distance
        if (Utils::compare_distance<float>((float)distance, 0))
        {
            if (logOutput)
            {
                m_stage.log()->get(logDEBUG4) << "0-query found: "
                                              << "index: " << indices_vec[i]
                                              << " distance: "
                                              << distances_vec[i] <<std::endl;
            }

            if (indices_vec[i] != -1)
                output.push_back(indices_vec[i]);

        }
        else
        {
            if (::sqrt(distances_vec[i]) < distance)
            {
                if (logOutput)
                {
                    m_stage.log()->get(logDEBUG4)   << "Query found: "
                                                    << "index: " << indices_vec[i]
                                                    << " distance: "
                                                    << distances_vec[i] <<std::endl;
                }
                if (indices_vec[i] != -1)
                    output.push_back(indices_vec[i]);
            }

        }
    }
#else
    boost::ignore_unused_variable_warning(x);
    boost::ignore_unused_variable_warning(y);
    boost::ignore_unused_variable_warning(z);
    boost::ignore_unused_variable_warning(distance);
    boost::ignore_unused_variable_warning(k);
#endif

    return output;
}


boost::uint32_t Index::readBufferImpl(PointBuffer& data)
{
    const boost::uint32_t numRead = getPrevIterator().read(data);


    m_stage.log()->floatPrecision(8);

    m_stage.log()->get(logDEBUG2) << "inserting data into index data array of capacity: " << data.getCapacity() << std::endl;

#ifdef PDAL_HAVE_FLANN

    bool logOutput = m_stage.log()->getLevel() > logDEBUG4;
    for (boost::uint32_t pointIndex=0; pointIndex<numRead; pointIndex++)
    {
        float x = static_cast<float>(getScaledValue(data, *m_xDim, pointIndex));
        float y = static_cast<float>(getScaledValue(data, *m_yDim, pointIndex));
        float z = static_cast<float>(getScaledValue(data, *m_zDim, pointIndex));

        m_data.push_back(x);
        m_data.push_back(y);
        if (m_stage.getNumDimensions() > 2)
        {
            m_data.push_back(z);
            if (logOutput)
            {
                m_stage.log()->get(logDEBUG4) << "adding z to index"  << z << std::endl;
            }

        }
        m_numIndexPoints++;
        if (logOutput)
        {
            m_stage.log()->get(logDEBUG4) << "index: "
                                          << pointIndex
                                          << " x: " << x
                                          << " y: " << y
                                          << " z: " << z << std::endl;
        }
    }
#endif

    return numRead;
}

double Index::getScaledValue(PointBuffer& data,
                             Dimension const& d,
                             std::size_t pointIndex) const
{
    double output(0.0);

    float flt(0.0);
    boost::int8_t i8(0);
    boost::uint8_t u8(0);
    boost::int16_t i16(0);
    boost::uint16_t u16(0);
    boost::int32_t i32(0);
    boost::uint32_t u32(0);
    boost::int64_t i64(0);
    boost::uint64_t u64(0);

    boost::uint32_t size = d.getByteSize();
    switch (d.getInterpretation())
    {
        case dimension::Float:
            if (size == 4)
            {
                flt = data.getField<float>(d, pointIndex);
                output = static_cast<double>(flt);
            }
            if (size == 8)
            {
                output = data.getField<double>(d, pointIndex);
            }
            break;

        case dimension::SignedInteger:
            if (size == 1)
            {
                i8 = data.getField<boost::int8_t>(d, pointIndex);
                output = d.applyScaling<boost::int8_t>(i8);
            }
            if (size == 2)
            {
                i16 = data.getField<boost::int16_t>(d, pointIndex);
                output = d.applyScaling<boost::int16_t>(i16);
            }
            if (size == 4)
            {
                i32 = data.getField<boost::int32_t>(d, pointIndex);
                output = d.applyScaling<boost::int32_t>(i32);
            }
            if (size == 8)
            {
                i64 = data.getField<boost::int64_t>(d, pointIndex);
                output = d.applyScaling<boost::int64_t>(i64);
            }
            break;

        case dimension::UnsignedInteger:
            if (size == 1)
            {
                u8 = data.getField<boost::uint8_t>(d, pointIndex);
                output = d.applyScaling<boost::uint8_t>(u8);
            }
            if (size == 2)
            {
                u16 = data.getField<boost::uint16_t>(d, pointIndex);
                output = d.applyScaling<boost::uint16_t>(u16);
            }
            if (size == 4)
            {
                u32 = data.getField<boost::uint32_t>(d, pointIndex);
                output = d.applyScaling<boost::uint32_t>(u32);
            }
            if (size == 8)
            {
                u64 = data.getField<boost::uint64_t>(d, pointIndex);
                output = d.applyScaling<boost::uint64_t>(u64);
            }
            break;

        case dimension::RawByte:
        case dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case dimension::Undefined:
            throw pdal_error("Dimension data type unable to be scaled in index filter");
    }

    return output;
}
void Index::build()
{

#ifdef PDAL_HAVE_FLANN

    // Build the index

    // flann::Logger::setLevel(flann::FLANN_LOG_DEBUG);
    // flann::Logger::setDestination("flannlog.log");

    boost::uint32_t num_dims = m_stage.getNumDimensions();
    m_dataset = new flann::Matrix<float>(&m_data[0], m_numIndexPoints, num_dims);
    m_stage.log()->get(logDEBUG2) << "Building index for size " << m_data.size()  <<" for points: " << m_stage.getNumPoints() <<std::endl;

    m_index = new flann::KDTreeSingleIndex<flann::L2_Simple<float> >(*m_dataset, flann::KDTreeIndexParams(4));

    m_index->buildIndex();
    m_stage.log()->get(logDEBUG2) << "Built index with memory: " << m_index->usedMemory() << " with size: " << m_index->size() << " veclen: " << m_index->veclen() << std::endl;

#endif
}

Index::~Index()
{
#ifdef PDAL_HAVE_FLANN
    if (m_index)
        delete m_index;

    if (m_dataset)
        delete m_dataset;
#endif
}

boost::uint64_t Index::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Index::atEndImpl() const
{
    return getPrevIterator().atEnd();
}


}
} // iterators::sequential


}
} // namespaces
