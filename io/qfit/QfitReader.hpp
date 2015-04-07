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

#include <memory>
#include <vector>

#include <pdal/Reader.hpp>
#include <pdal/Options.hpp>
#include <pdal/util/IStream.hpp>


extern "C" int32_t QfitReader_ExitFunc();
extern "C" PF_ExitFunc QfitReader_InitPlugin();

namespace pdal
{

enum QFIT_Format_Type
{
    QFIT_Format_10 = 10,
    QFIT_Format_12 = 12,
    QFIT_Format_14 = 14,
    QFIT_Format_Unknown = 128
};

class qfit_error : public pdal_error
{
public:

    qfit_error(std::string const& msg)
        : pdal_error(msg)
    {}
};

class PDAL_DLL QfitReader : public pdal::Reader
{
public:
    QfitReader();

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();
    static Dimension::IdList getDefaultDimensions();

    std::string getFileName() const;

    std::size_t getPointDataOffset() const
        { return m_offset; }
    uint32_t getPointDataSize() const
        { return m_size; }
    point_count_t getNumPoints() const
        { return m_numPoints; }
    bool eof()
        { return m_index >= m_numPoints; }

    // this is called by the stage's iterator
    point_count_t processBuffer(PointViewPtr view, std::istream& stream,
        point_count_t count) const;

private:
    QFIT_Format_Type m_format;
    std::ios::off_type m_point_bytes;
    std::size_t m_offset;
    uint32_t m_size;
    bool m_flip_x;
    double m_scale_z;
    bool m_littleEndian;
    point_count_t m_numPoints;
    std::unique_ptr<IStream> m_istream;
    point_count_t m_index;

    virtual void processOptions(const Options& ops);
    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr buf, point_count_t count);
    virtual void done(PointTableRef table);

    QfitReader& operator=(const QfitReader&); // not implemented
    QfitReader(const QfitReader&); // not implemented
};

} // namespace pdal
