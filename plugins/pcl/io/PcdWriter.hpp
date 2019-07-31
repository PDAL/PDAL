/******************************************************************************
* Copyright (c) 2014, Brad Chambers (brad.chambers@gmail.com)
* Copytight (c) 2016, Logan Byers (logan.c.byers@gmail.com)
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

#pragma once

#include <pdal/EigenUtils.hpp>
#include <pdal/Writer.hpp>
#include <pdal/util/FileUtils.hpp>

#include "../PCLConversions.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>

#include <vector>
#include <string>

namespace pdal
{

class PDAL_DLL PcdWriter : public Writer
{
public:
    PcdWriter()
    {}
    std::string getName() const;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void write(const PointViewPtr view);
    virtual void done(PointTableRef table);

    template<typename CloudT>
    inline void writeView(const PointViewPtr view); // implemented in header

    std::string m_filename;
    std::string m_compression_string;
    uint8_t m_compression;
    bool m_xyz;
    bool m_subtract_minimum;
    double m_offset_x;
    double m_offset_y;
    double m_offset_z;
    double m_scale_x;
    double m_scale_y;
    double m_scale_z;

    PcdWriter& operator=(const PcdWriter&); // not implemented
    PcdWriter(const PcdWriter&); // not implemented
};


template<typename CloudT>
void PcdWriter::writeView(const PointViewPtr view)
{
    typedef typename CloudT::PointType PointT;
    typename CloudT::Ptr cloud(new CloudT);
    BOX3D bounds;
    if (m_subtract_minimum)
    {
        calculateBounds(*view, bounds);
        bounds.grow(bounds.minx + m_offset_x,
                    bounds.miny + m_offset_y,
                    bounds.minz + m_offset_z);
    }
    else
    {
        bounds.grow(m_offset_x, m_offset_y, m_offset_z);
    }
    pclsupport::PDALtoPCD(view, *cloud, bounds, m_scale_x, m_scale_y, m_scale_z);
    pcl::PCDWriter w;

    if (m_compression_string == "binary")
    {
      m_compression = 1;
    }
    else if (m_compression_string == "compressed")
    {
      m_compression = 2;
    }
    else  // including "ascii"
    {
      m_compression = 0;
    }

    switch (m_compression)
    {
        case 0 : w.writeASCII<PointT>(m_filename, *cloud); break;
        case 1 : w.writeBinary<PointT>(m_filename, *cloud); break;
        case 2 : w.writeBinaryCompressed<PointT>(m_filename, *cloud); break;
    }
}


} // namespaces
