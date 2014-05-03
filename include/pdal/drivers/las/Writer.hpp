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

#include <pdal/Writer.hpp>
#include <pdal/drivers/las/Support.hpp>
#include <pdal/drivers/las/Header.hpp>
#include <pdal/drivers/las/SummaryData.hpp>
#include <pdal/StreamFactory.hpp>
#include <boost/scoped_ptr.hpp>


namespace pdal
{
namespace drivers
{
namespace las
{


//
// supported options:
//   <uint32>id
//   <bool>debug
//   <uint32>verbose
//   <string>a_srs
//   <bool>compression
//   <string>filename  [required]
//
class PDAL_DLL Writer : public pdal::Writer
{
public:
    SET_STAGE_NAME("drivers.las.writer", "Las Writer")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.las.writer.html")
    SET_STAGE_ENABLED(true)


    Writer(const Options&);
    Writer(std::ostream*);
    virtual ~Writer();

    virtual void initialize();
    static Options getDefaultOptions();

    void setFormatVersion(boost::uint8_t majorVersion, boost::uint8_t minorVersion);
    void setPointFormat(PointFormat);
    void setDate(boost::uint16_t dayOfYear, boost::uint16_t year);

    void setProjectId(const boost::uuids::uuid&);

    // up to 32 chars (default is "PDAL")
    void setSystemIdentifier(const std::string& systemId);

    // up to 32 chars (default is "PDAL x.y.z")
    void setGeneratingSoftware(const std::string& softwareId);

    void setHeaderPadding(boost::uint32_t const& v);

    // default false
    void setCompressed(bool);

protected:
    virtual void writeBegin(boost::uint64_t targetNumPointsToWrite);
    virtual void writeBufferBegin(PointBuffer const&);
    virtual boost::uint32_t writeBuffer(const PointBuffer&);
    virtual void writeBufferEnd(PointBuffer const&);
    virtual void writeEnd(boost::uint64_t actualNumPointsWritten);

    OutputStreamManager m_streamManager;

private:
    LasHeader m_lasHeader;
    boost::uint32_t m_numPointsWritten;
    SummaryData m_summaryData;

#ifdef PDAL_HAVE_LASZIP
    boost::scoped_ptr<LASzipper> m_zipper;
    boost::scoped_ptr<ZipPoint> m_zipPoint;
#endif

    bool m_headerInitialized;
    boost::uint64_t m_streamOffset; // the first byte of the LAS file
	void setOptions();
    bool doForwardThisMetadata(std::string const& name) const;
    void setVLRsFromMetadata(LasHeader& header, Metadata const& metadata, Options const& opts);
    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented


    template<typename T>
    T getMetadataOption(   pdal::Options const& options, 
                                    pdal::Metadata const& metadata, 
                                    std::string const& name,
                                    T default_value) const
    {
        boost::optional<std::string> candidate = options.getMetadataOption<std::string>(name);
        if (!candidate)
        {
            return options.getValueOrDefault<T>(name, default_value);
        }
            
        if (boost::algorithm::iequals(*candidate, "FORWARD"))
        {
            boost::optional<std::string> m =
                metadata.getValueOptional<std::string>(name);
            if (m)
            {
                T v = boost::lexical_cast<T>(*m) ;                
                return v;
            }
        } 
        else
        {
            T v = boost::lexical_cast<T>(*candidate) ;
            return v;
        }
        return default_value;
    }    
};

}
}
} // namespaces

#endif
