/******************************************************************************
* Copyright (c) 2011, Brad Chambers (brad.chambers@gmail.com)
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

#include <pdal/drivers/pcd/Writer.hpp>

#include <algorithm>
#include <iostream>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/erase.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>

#include <pdal/PCLConversions.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/drivers/pcd/point_types.hpp>

namespace pdal
{
namespace drivers
{
namespace pcd
{


void PcdWriter::processOptions(const Options& ops)
{
    m_filename = ops.getValueOrThrow<std::string>("filename");
    m_compressed = ops.getValueOrDefault("compression", false);
}


Options PcdWriter::getDefaultOptions()
{
    Options options;

    options.add("filename", "", "Filename to write PCD file to");
    options.add("compression", false, "Write binary compressed data?");

    return options;
}


void PcdWriter::write(const PointBuffer& data)
{
    pcl::PointCloud<XYZIRGBA>::Ptr cloud(new pcl::PointCloud<XYZIRGBA>);
    Bounds<double> const& buffer_bounds = data.calculateBounds();
    pdal::PDALtoPCD(const_cast<PointBuffer&>(data), *cloud, buffer_bounds);

    pcl::PCDWriter w;

    if (m_compressed)
        w.writeBinaryCompressed<XYZIRGBA>(m_filename, *cloud);
    else
        w.writeASCII<XYZIRGBA>(m_filename, *cloud);
}


}
}
} // namespaces
