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

#pragma once

#include <pdal/Writer.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/StageFactory.hpp>

#include "PCLConversions.hpp"

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

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();

private:
    virtual void processOptions(const Options&);
    virtual void write(const PointViewPtr view);

    template<typename CloudT>
    inline void writeView(const PointViewPtr view); // implemented in header

    std::string m_filename;
    bool m_compressed;
    bool m_binary;
    bool m_xyz;
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
    bounds.grow(m_offset_x, m_offset_y, m_offset_z);
    pclsupport::PDALtoPCD(view, *cloud, bounds, m_scale_x, m_scale_y, m_scale_z);
    pcl::PCDWriter w;
    if (m_compressed)
    {
        w.writeBinaryCompressed<PointT>(m_filename, *cloud);
    }
    else if (m_binary)
    {
        w.writeBinary<PointT>(m_filename, *cloud);
    }
    else
    {
        w.writeASCII<PointT>(m_filename, *cloud);
    }
}


} // namespaces
