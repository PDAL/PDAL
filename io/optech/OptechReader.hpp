/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski <pete.gadomski@gmail.com>
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

#include <memory>
#include <vector>

#include <pdal/Reader.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/Extractor.hpp>
#include <pdal/util/Georeference.hpp>
#include <pdal/util/IStream.hpp>
#include <pdal/pdal_export.hpp>

#include "OptechCommon.hpp"

extern "C" int32_t OptechReader_ExitFunc();
extern "C" PF_ExitFunc OptechReader_InitPlugin();

namespace pdal
{


class PDAL_DLL OptechReader : public Reader
{
public:
    static void *create();
    static int32_t destroy(void *);
    std::string getName() const;

    static const size_t MaximumNumberOfReturns = 4;
    static const size_t NumBytesInRecord = 69;
    static const size_t MaxNumRecordsInBuffer = 1e6 / NumBytesInRecord;

    OptechReader();

    static Dimension::IdList getDefaultDimensions();
    const CsdHeader& getHeader() const;

private:
    typedef std::vector<char> buffer_t;
    typedef buffer_t::size_type buffer_size_t;

    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t num);
    size_t fillBuffer();
    virtual void done(PointTableRef table);

    CsdHeader m_header;
    georeference::RotationMatrix m_boresightMatrix;
    std::unique_ptr<IStream> m_istream;
    buffer_t m_buffer;
    LeExtractor m_extractor;
    size_t m_recordIndex;
    size_t m_returnIndex;
    CsdPulse m_pulse;
};
}
