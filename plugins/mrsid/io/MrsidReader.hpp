/******************************************************************************
* Copyright (c) 2011, Michael S. Rosen (michael.rosen@gmail.com)
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

#include <pdal/Reader.hpp>

#include <pdal/util/Bounds.hpp>

#include <lidar/PointSource.h>
#include <lidar/PointData.h>
#include <lidar/MG4PointReader.h>


extern "C" int32_t MrsidReader_ExitFunc();
extern "C" PF_ExitFunc MrsidReader_InitPlugin();


namespace pdal
{


// The MrSIDReader wraps LT's PointSource abstraction
//
class PDAL_DLL MrsidReader : public pdal::Reader
{

public:
    virtual ~MrsidReader(){};
    MrsidReader();

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();

    point_count_t getNumPoints() const
        { if (m_PS)
            return m_PS->getNumPoints();
          else
            return 0;
        }

protected:
    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr layout);
private:
    LizardTech::MG4PointReader *m_PS;
    LizardTech::PointIterator *m_iter;
    LizardTech::PointInfo m_pointInfo;
    PointLayoutPtr m_layout;

    point_count_t m_index;

    MrsidReader& operator=(const MrsidReader&); // not implemented
    MrsidReader(const MrsidReader&); // not implemented

    void LayoutToPointInfo(const PointLayout &layout, LizardTech::PointInfo &pointInfo) const;
    virtual QuickInfo inspect();
    virtual void ready(PointTableRef table)
        { ready(table, m_metadata); }
    virtual void ready(PointTableRef table, MetadataNode& m);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual void done(PointTableRef table);
    virtual bool eof()
        { return m_index >= getNumPoints(); }
    bool m_initialized;
};


} // namespaces
