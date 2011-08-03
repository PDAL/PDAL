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

#ifndef INCLUDED_DRIVERS_LAS_WRITER_HPP
#define INCLUDED_DRIVERS_LAS_WRITER_HPP

#include <pdal/pdal.hpp>

#include <pdal/Writer.hpp>
#include <pdal/drivers/las/Header.hpp>
#include <pdal/drivers/las/SummaryData.hpp>
#include <pdal/StreamManager.hpp>
#include <boost/scoped_ptr.hpp>

// liblaszip
class LASzip;
class LASzipper;

namespace pdal { namespace drivers { namespace las {

class ZipPoint;

class PDAL_DLL LasWriter : public Writer
{
    DECLARE_STATICS

public:
    LasWriter(Stage& prevStage, const Options&);
    LasWriter(Stage& prevStage, std::ostream*);
    ~LasWriter();

    virtual void initialize();

    void setFormatVersion(boost::uint8_t majorVersion, boost::uint8_t minorVersion);
    void setPointFormat(PointFormat);
    void setDate(boost::uint16_t dayOfYear, boost::uint16_t year);
    
    void setProjectId(const boost::uuids::uuid&);

    // up to 32 chars (default is "PDAL")
    void setSystemIdentifier(const std::string& systemId); 
    
    // up to 32 chars (default is "PDAL x.y.z")
    void setGeneratingSoftware(const std::string& softwareId);

    // default false
    void setCompressed(bool);

    void setSpatialReference(const SpatialReference&);

protected:
    virtual void writeBegin(boost::uint64_t targetNumPointsToWrite);
    virtual boost::uint32_t writeBuffer(const PointBuffer&);
    virtual void writeEnd(boost::uint64_t actualNumPointsWritten);

private:
    OStreamManager m_streamManager;

    LasHeader m_lasHeader;
    boost::uint32_t m_numPointsWritten;
    bool m_isCompressed;
    SummaryData m_summaryData;
    SpatialReference m_spatialReference;

#ifdef PDAL_HAVE_LASZIP
    boost::scoped_ptr<LASzip> m_zip;
    boost::scoped_ptr<LASzipper> m_zipper;
    boost::scoped_ptr<ZipPoint> m_zipPoint;
#endif

    LasWriter& operator=(const LasWriter&); // not implemented
    LasWriter(const LasWriter&); // not implemented
};

} } } // namespaces

#endif
