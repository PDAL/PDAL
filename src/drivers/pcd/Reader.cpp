/******************************************************************************
* Copyright (c) 2014, Brad Chambers (brad.chambers@gmail.com)
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

#include <pdal/drivers/pcd/Reader.hpp>

#include <boost/algorithm/string.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/PCLConversions.hpp>
#include <pdal/drivers/pcd/point_types.hpp>

namespace pdal
{
namespace drivers
{
namespace pcd
{


void PcdReader::ready(PointContext ctx)
{
    pcl::PCLPointCloud2 cloud;
    pcl::PCDReader r;
    r.readHeader(m_filename, cloud);
    m_numPts = cloud.height * cloud.width;
}


Options PcdReader::getDefaultOptions()
{
    Options options;
    return options;
}


void PcdReader::addDimensions(PointContext ctx)
{
    ctx.registerDims(getDefaultDimensions());
}


void PcdReader::processOptions(const Options& options)
{
    m_filename = options.getOption("filename").getValue<std::string>();
}


StageSequentialIterator* PcdReader::createSequentialIterator() const
{
    return new iterators::sequential::PcdSeqIterator(getDefaultDimensions(), m_filename);
}

namespace iterators
{
namespace sequential
{

point_count_t PcdSeqIterator::readImpl(PointBuffer& data, std::string filename)
{
    pcl::PointCloud<XYZIRGBA>::Ptr cloud(new pcl::PointCloud<XYZIRGBA>);

    pcl::PCDReader r;
    r.read<XYZIRGBA>(filename, *cloud);

    pdal::PCDtoPDAL(*cloud, data);

    point_count_t numPoints = cloud->points.size();

    return numPoints;
}

} // sequential
} // iterators

}
}
} // namespaces

