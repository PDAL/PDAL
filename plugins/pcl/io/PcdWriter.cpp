/******************************************************************************
* Copyright (c) 2011, Brad Chambers (brad.chambers@gmail.com)
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

#include "PcdWriter.hpp"
#include "point_types.hpp"

#include <algorithm>
#include <iostream>
#include <map>

#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>

#include "../PCLConversions.hpp"

#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "writers.pcd",
    "Write data in the Point Cloud Library (PCL) format.",
    "http://pdal.io/stages/writers.pcd.html"
};

CREATE_SHARED_STAGE(PcdWriter, s_info)

std::string PcdWriter::getName() const { return s_info.name; }


void PcdWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "PCD output filename", m_filename).setPositional();
    args.add("compression", "Level of PCD compression to use "
        "(ascii, binary, compressed)", m_compression_string);
    args.add("xyz", "Write only XYZ dimensions?", m_xyz, false);
    args.add("subtract_minimum", "Set origin to minimum of XYZ dimension",
        m_subtract_minimum, true);
    args.add("offset_x", "Offset to be subtracted from XYZ position",
        m_offset_x, 0.0);
    args.add("offset_y", "Offset to be subtracted from XYZ position",
        m_offset_y, 0.0);
    args.add("offset_z", "Offset to be subtracted from XYZ position",
        m_offset_z, 0.0);
    args.add("scale_x", "Scale to divide from XYZ dimension", m_scale_x, 1.0);
    args.add("scale_y", "Scale to divide from XYZ dimension", m_scale_y, 1.0);
    args.add("scale_z", "Scale to divide from XYZ dimension", m_scale_z, 1.0);
}


void PcdWriter::write(const PointViewPtr view)
{
    if (m_xyz)
    {
        writeView<pcl::PointCloud<pcl::PointXYZ> >(view);
    }
    else
    {
        writeView<pcl::PointCloud<XYZIRGBA> >(view);
    }
}


void PcdWriter::done(PointTableRef)
{
    getMetadata().addList("filename", m_filename);
}


} // namespaces
