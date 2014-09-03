/************************************************************************
 * Copyright (c) 2012, CARIS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of CARIS nor the names of its contributors may be
 *     used to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************/

#include <pdal/pdal_internal.hpp>
#include <pdal/drivers/caris/CloudReader.hpp>

#include "Utils.hpp"


#ifdef _MSC_VER
#   pragma warning(push, 3)
#   pragma warning(disable : DISABLED_3RDPARTY_WARNINGS)
#endif

#include <pdal/Dimension.hpp>

#ifdef _MSC_VER
#   pragma warning(pop)
// decorated name length exceeded, name was truncated
#   pragma warning(disable : 4503)
#endif

namespace pdal
{
namespace csar
{

namespace
{

void logCallback(void *in_reader, const char *in_message)
{
    CloudReader *reader = (CloudReader *)in_reader;
    if (reader)
        reader->log()->get(LogLevel::Info) << in_message << std::flush;
}

} // anonymous namespace


CloudReader::~CloudReader()
{
    if (m_cloud)
        caris_cloud_release(m_cloud);
}


point_count_t CloudReader::numPoints()
{
    return caris_cloud_num_points(m_cloud);
}


void CloudReader::initialize()
{
    int status = caris_cloud_open(getURI().c_str(), &m_cloud,
        &logCallback, this);
    if (status)
    {
        std::string msg = caris_status_message(status, isDebug());
        throw pdal::pdal_error(msg);
    }
}


void CloudReader::addDimensions(PointContext ctx)
{
    int numDims = 0;
    const caris_dimension *dimArray = NULL;
    caris_cloud_dimensions(m_cloud, &dimArray, &numDims);

    // caris_dimesions may contain a tuple of numeric elements which need
    // to be mapped to multiple pdal dimenions
    for (int dimIndex = 0; dimIndex < numDims; ++dimIndex)
    {
        caris_dimension const& carisDim = dimArray[dimIndex];

        for (int tupleIndex = 0; tupleIndex < carisDim.tuple_length;
            ++tupleIndex)
        {
            Dimension::Id::Enum dim = Dimension::Id::Unknown;
            Dimension::Type::Enum inType = Dimension::Type::None;
            std::string name = carisDim.name;
            if (carisDim.tuple_length > 1)
            {
                if (dimIndex == 0 && carisDim.type == CARIS_TYPE_FLOAT64 &&
                    carisDim.tuple_length == 3)
                {
                    Dimension::Id::Enum xyz[] = 
                        { Dimension::Id::X,
                          Dimension::Id::Y,
                          Dimension::Id::Z };
                    inType = Dimension::Type::Double;
                    dim = xyz[tupleIndex];
                }
                else
                    name += "." + boost::lexical_cast<std::string>(tupleIndex);
            }

            if (dim != Dimension::Id::Unknown)
            {
                inType = utils::carisTypeToPdal((caris_type)carisDim.type);
                if (inType == Dimension::Type::None)
                    continue;
                dim = ctx.registerOrAssignDim(name, inType);
            }
            m_dims[dim] = DimInfo(dimIndex, tupleIndex, inType, &carisDim);
        }
    }

    if (caris_cloud_status(m_cloud))
    {
        std::string msg = caris_status_message(caris_cloud_status(m_cloud),
            isDebug());
        throw pdal::pdal_error(msg);
    }
}

using namespace csar::utils;

void CloudReader::ready(PointContext ctx)
{
    m_itr = caris_cloud_create_itr(m_cloud);
    throwIfItrError();
}


void CloudReader::done(PointContext ctx)
{
    if (m_itr)
        caris_itr_release(m_itr);
}


point_count_t CloudReader::read(pdal::PointBuffer& io_buffer, point_count_t num)
{
    assert(m_itr);

    if (eof())
        return 0;

    // Determine the number of points availabe from the current iterator.
    point_count_t numPoints = caris_itr_num_points(m_itr);
    numPoints -= m_currentOffset;

    // We don't want any more points than requested.
    num = std::min(numPoints, num);

    PointId startIdx = io_buffer.size();
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        Dimension::Id::Enum dim = di->first;
        DimInfo& dimInfo = di->second;

        int32_t numDimElements = 0;
        const void *dimElements = caris_itr_read(m_itr, dimInfo.dimIndex,
            &numDimElements);
        throwIfItrError();
        assert((point_count_t)numDimElements == num);

        size_t elementSize = Dimension::size(dimInfo.type);
        size_t srcStride = dimInfo.dimension->tuple_length * elementSize;
        uint8_t *src = (uint8_t *)dimElements +
            (m_currentOffset * srcStride) +
            (elementSize * dimInfo.tupleIndex);

        PointId idx = startIdx;
        for (size_t i = 0; i < num; ++i, ++idx, src += srcStride)
            io_buffer.setField(dim, dimInfo.type, idx, (void *)src);
    }
    m_currentOffset += num;

    if ((point_count_t)m_currentOffset == numPoints)
    {
        m_currentOffset = 0;
        caris_itr_next(m_itr);
    }

    throwIfItrError();

    return num;
}


bool CloudReader::eof()
{
    return (bool)caris_itr_done(m_itr);
}


//************************************************************************
//! throw a pdal::pdal_error if an error has occured in the wrapped caris_itr
//************************************************************************
void CloudReader::throwIfItrError() const
{
    if (int status = caris_itr_status(m_itr))
    {
        std::string msg = caris_status_message(status, isDebug());
        throw pdal::pdal_error(msg);
    }
}

} // namespace csar
} // namespace pdal
