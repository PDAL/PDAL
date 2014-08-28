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

#include <pdal/Writer.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/StageFactory.hpp>

#include <vector>
#include <string>

namespace pdal
{
namespace drivers
{
namespace text
{

#ifdef USE_PDAL_PLUGIN_TEXT
PDAL_C_START

PDAL_DLL void PDALRegister_writer_text(void* factory);

PDAL_C_END
#endif

typedef std::shared_ptr<std::ostream> FileStreamPtr;

class PDAL_DLL Writer : public pdal::Writer
{
public:
    SET_STAGE_NAME("drivers.text.writer", "Text Writer")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.text.writer.html")
    SET_STAGE_ENABLED(true)

    Writer(const Options& options) : pdal::Writer(options)
    {}

    static Options getDefaultOptions();

private:
    virtual void processOptions(const Options&);
    virtual void ready(PointContext ctx);
    virtual void write(const PointBuffer& buf);
    virtual void done(PointContext ctx);

    void writeHeader(PointContext ctx);
    void writeFooter();
    void writeGeoJSONHeader();
    void writeCSVHeader(PointContext ctx);
    void writePCDHeader();

    void writeGeoJSONBuffer(const PointBuffer& data);
    void writeCSVBuffer(const PointBuffer& data);
    void writePCDBuffer(const PointBuffer& data);
    
    std::string m_filename;
    std::string m_outputType;
    std::string m_callback;
    bool m_writeAllDims;
    std::string m_dimOrder;
    bool m_writeHeader;
    std::string m_newline;
    std::string m_delimiter;
    bool m_quoteHeader;
    bool m_packRgb;

    FileStreamPtr m_stream;
    Dimension::IdList m_dims;

    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented
};

} // namespace text
} // namespace drivers
} // namespace pdal

