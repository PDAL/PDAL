/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#include <pdal/Options.hpp>
#include <pdal/Reader.hpp>
#include <pdal/util/IStream.hpp>

#include <memory>
#include <vector>

extern "C" int32_t TerrasolidReader_ExitFunc();
extern "C" PF_ExitFunc TerrasolidReader_InitPlugin();

namespace pdal
{


enum TERRASOLID_Format_Type
{
    TERRASOLID_Format_1 = 20010712,
    TERRASOLID_Format_2 = 20020715,
    TERRASOLID_Format_Unknown = 999999999
};

struct TerraSolidHeader
{
    TerraSolidHeader() :
        HdrSize(0),
        HdrVersion(0),
        RecogVal(0),
        PntCnt(0),
        Units(0),
        OrgX(0),
        OrgY(0),
        OrgZ(0),
        Time(0),
        Color(0)
    {}

    int32_t HdrSize;
    int32_t HdrVersion;
    int32_t RecogVal;
    char RecogStr[4];
    int32_t PntCnt;
    int32_t Units;
    double OrgX;
    double OrgY;
    double OrgZ;
    int32_t Time;
    int32_t Color;
};

typedef std::unique_ptr<TerraSolidHeader> TerraSolidHeaderPtr;
class terrasolid_error : public pdal_error
{
public:

    terrasolid_error(std::string const& msg)
        : pdal_error(msg)
    {}
};

class PDAL_DLL TerrasolidReader : public pdal::Reader
{
public:
    TerrasolidReader() : pdal::Reader(),
        m_format(TERRASOLID_Format_Unknown)
    {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    static Dimension::IdList getDefaultDimensions();

    point_count_t getNumPoints() const
        { return m_header->PntCnt; }

    const TerraSolidHeader& getHeader() const { return *m_header; }

    // this is called by the stage's iterator
    uint32_t processBuffer(PointViewPtr view, std::istream& stream,
        uint64_t numPointsLeft) const;

private:
    TerraSolidHeaderPtr m_header;
    TERRASOLID_Format_Type m_format;
    uint32_t m_size;
    bool m_haveColor;
    bool m_haveTime;
    uint32_t m_baseTime;
    std::unique_ptr<IStream> m_istream;
    point_count_t m_index;

    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual void done(PointTableRef table);
    virtual bool eof()
        { return m_index >= getNumPoints(); }

    TerrasolidReader& operator=(const TerrasolidReader&); // not implemented
    TerrasolidReader(const TerrasolidReader&); // not implemented
};

} // namespace pdal
