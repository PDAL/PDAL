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

#ifndef INCLUDED_DRIVERS_TEXT_WRITER_HPP
#define INCLUDED_DRIVERS_TEXT_WRITER_HPP

#include <pdal/Writer.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/StageFactory.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tokenizer.hpp>

#include <vector>
#include <string>

// pdal::Writer* createTextWriter(pdal::Stage& prevStage, const pdal::Options& options);


namespace pdal
{
namespace drivers
{
namespace text
{

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

class text_driver_error : public pdal_error
{
public:
    text_driver_error(std::string const& msg)
        : pdal_error(msg)
    {}
};

#ifdef USE_PDAL_PLUGIN_TEXT
PDAL_C_START

PDAL_DLL void PDALRegister_writer_text(void* factory);

PDAL_C_END
#endif

typedef boost::shared_ptr<std::ostream> FileStreamPtr;

class PDAL_DLL Writer : public pdal::Writer
{
public:
    SET_STAGE_NAME("drivers.text.writer", "Text Writer")

    Writer(Stage& prevStage, const Options&);
    ~Writer();

    virtual void initialize();
    static Options getDefaultOptions();

protected:
    virtual void writeBegin(boost::uint64_t targetNumPointsToWrite);
    virtual boost::uint32_t writeBuffer(const PointBuffer&);
    virtual void writeEnd(boost::uint64_t actualNumPointsWritten);

private:

    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented

    std::string getStringRepresentation(PointBuffer const& data,
                                        Dimension const& d,
                                        std::size_t pointIndex) const;

    void WriteHeader(pdal::Schema const& schema);
    
    void WriteGeoJSONHeader(pdal::Schema const& schema);
    void WriteCSVHeader(pdal::Schema const& schema);
    void WritePCDHeader(pdal::Schema const& schema);
    
    void WriteCSVBuffer(const PointBuffer& data);
    void WriteGeoJSONBuffer(const PointBuffer& data);
    void WritePCDBuffer(const PointBuffer& data);
    
    std::vector<boost::tuple<std::string, std::string> >  getDimensionOrder(pdal::Schema const& schema) const;
    FileStreamPtr m_stream;
    bool bWroteHeader;
    bool bWroteFirstPoint;
};

}
}
} // namespaces

#endif
