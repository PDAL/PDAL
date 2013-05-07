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

#ifndef INCLUDED_DRIVERS_PRC_WRITER_HPP
#define INCLUDED_DRIVERS_PRC_WRITER_HPP

#include <pdal/drivers/prc/oPRCFile.hpp>
#include <pdal/Writer.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/StageFactory.hpp>

#include <boost/cstdint.hpp>
//#include <boost/scoped_ptr.hpp>
//#include <boost/tuple/tuple.hpp>

//#include <vector>
#include <string>


namespace pdal
{
namespace drivers
{
namespace prc
{


class prc_driver_error : public pdal_error
{
public:
    prc_driver_error(std::string const& msg)
        : pdal_error(msg)
    {}
};

/*
PDAL_C_START

PDAL_DLL void PDALRegister_writer_prc(void* factory);

PDAL_C_END
*/

class PDAL_DLL Writer : public pdal::Writer
{
public:
    SET_STAGE_NAME("drivers.prc.writer", "PRC Writer")

    Writer(Stage& prevStage, const Options&);
    ~Writer();

    virtual void initialize();
    static Options getDefaultOptions();

protected:
    virtual void writeBegin(boost::uint64_t targetNumPointsToWrite);
    virtual boost::uint32_t writeBuffer(const PointBuffer&);
    virtual void writeEnd(boost::uint64_t actualNumPointsWritten);

    oPRCFile m_prcFile;
    PRCoptions grpopt;

private:

    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented

};

}
}
} // namespaces

#endif
