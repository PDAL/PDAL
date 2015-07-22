/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
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

#include <map>

#include <pdal/KDIndex.hpp>
#include <pdal/Kernel.hpp>
#include <pdal/PointView.hpp>

extern "C" int32_t DeltaKernel_ExitFunc();
extern "C" PF_ExitFunc DeltaKernel_InitPlugin();

namespace pdal
{

struct DimIndex
{
    std::string m_name;
    Dimension::Id::Enum m_srcId;
    Dimension::Id::Enum m_candId;
    double m_min;
    double m_max;
    double m_avg;
    point_count_t m_cnt;

    DimIndex() : m_srcId(Dimension::Id::Unknown),
        m_candId(Dimension::Id::Unknown),
        m_min((std::numeric_limits<double>::max)()),
        m_max((std::numeric_limits<double>::lowest)()),
        m_avg(0.0), m_cnt(0)
    {}
};
typedef std::map<std::string, DimIndex> DimIndexMap;

class PDAL_DLL DeltaKernel : public Kernel
{
public:
    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;
    int execute();

private:
    DeltaKernel();
    void addSwitches();
    PointViewPtr loadSet(const std::string& filename, PointTable& table);
    MetadataNode dump(PointViewPtr& srcView, PointViewPtr& candView,
        KD3Index& index, DimIndexMap& dims);
    MetadataNode dumpDetail(PointViewPtr& srcView, PointViewPtr& candView,
        KD3Index& index, DimIndexMap& dims);
    void accumulate(DimIndex& d, double v);

    std::string m_sourceFile;
    std::string m_candidateFile;
    std::string m_outputFile;

    /**
    std::ostream* m_outputStream;
    **/

    bool m_3d;
    bool m_detail;
    bool m_allDims;
};

} // namespace pdal

