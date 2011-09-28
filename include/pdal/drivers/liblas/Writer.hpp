/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#ifndef INCLUDED_DRIVERS_LIBLAS_LIBLASWRITER_HPP
#define INCLUDED_DRIVERS_LIBLAS_LIBLASWRITER_HPP

#include <pdal/pdal.hpp>

#include <pdal/Writer.hpp>
#include <pdal/StreamManager.hpp>
#include <pdal/drivers/las/Support.hpp>
#include <pdal/drivers/las/SummaryData.hpp>

#include <liblas/liblas.hpp>
#include <liblas/guid.hpp>

#include <pdal/external/boost/uuid/uuid.hpp>


namespace pdal { namespace drivers { namespace liblas {

//
// supported options:
//   <uint32>id
//   <bool>debug
//   <uint32>verbose
//   <string>a_srs
//   <bool>compression
//   <string>filename  [required]
//

// we default to LAS 1.2, point format 0
class PDAL_DLL Writer : public pdal::Writer
{
public:
    SET_STAGE_NAME("drivers.liblas.writer", "Liblas Writer")

    Writer(Stage& prevStage, const Options&);
    Writer(Stage& prevStage, std::ostream*);
    ~Writer();

    virtual void initialize();
    virtual const Options getDefaultOptions() const;

    void setFormatVersion(boost::uint8_t majorVersion, boost::uint8_t minorVersion);
    void setPointFormat(::pdal::drivers::las::PointFormat);
    void setDate(boost::uint16_t dayOfYear, boost::uint16_t year);
    
    void setProjectId(const pdal::external::boost::uuids::uuid&);

    // up to 32 chars (default is "PDAL")
    void setSystemIdentifier(const std::string& systemId); 
    
    // up to 32 chars (default is "PDAL x.y.z")
    void setGeneratingSoftware(const std::string& softwareId);

    // default false
    void setCompressed(bool);
    
    void setHeaderPadding(boost::uint32_t const& v);

    // for dumping
    virtual boost::property_tree::ptree toPTree() const;

protected:
    virtual void writeBegin(boost::uint64_t targetNumPointsToWrite);
    virtual boost::uint32_t writeBuffer(const PointBuffer&);
    virtual void writeEnd(boost::uint64_t actualNumPointsWritten);

private:
    void setupExternalHeader();

    OStreamManager m_ostreamManager;
    ::liblas::Writer* m_externalWriter;
    ::liblas::HeaderPtr m_externalHeader;

    ::pdal::drivers::las::SummaryData m_summaryData;

    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented
};

} } } // namespaces

#endif
