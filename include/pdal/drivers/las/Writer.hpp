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

#pragma once

#include <pdal/Writer.hpp>
#include <pdal/drivers/las/Support.hpp>
#include <pdal/drivers/las/Header.hpp>
#include <pdal/drivers/las/SummaryData.hpp>
#include <pdal/StreamFactory.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>

namespace pdal
{
namespace drivers
{

namespace nitf
{
    class Writer;
}

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
    friend class nitf::Writer;
public:
    SET_STAGE_NAME("drivers.las.writer", "Las Writer")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.las.writer.html")
    SET_STAGE_ENABLED(true)

    Writer(const Options&);
    Writer(std::ostream*);
    Writer(const Options&, std::ostream*);
    virtual ~Writer();

    static Options getDefaultOptions();

    void setFormatVersion(uint8_t majorVersion, uint8_t minorVersion);
    void setPointFormat(PointFormat);
    void setDate(uint16_t dayOfYear, uint16_t year);

    void setProjectId(const boost::uuids::uuid&);

    // up to 32 chars (default is "PDAL")
    void setSystemIdentifier(const std::string& systemId);

    // up to 32 chars (default is "PDAL x.y.z")
    void setGeneratingSoftware(const std::string& softwareId);

    void setHeaderPadding(boost::uint32_t const& v);

    // default false
    void setCompressed(bool);

protected:
    void Construct();
    virtual void initialize();

    OutputStreamManager m_streamManager;

private:
    LasHeader m_lasHeader;
    boost::uint32_t m_numPointsWritten;
    SummaryData m_summaryData;

#ifdef PDAL_HAVE_LASZIP
    boost::scoped_ptr<LASzipper> m_zipper;
    boost::scoped_ptr<ZipPoint> m_zipPoint;
#endif
    virtual void processOptions(const Options& options);
    virtual void ready(PointContext ctx);
    virtual void write(const PointBuffer& pointBuffer);
    virtual void done(PointContext ctx);
    bool m_headerInitialized;
    bool m_discardHighReturnNumbers;
    boost::uint64_t m_streamOffset; // the first byte of the LAS file
	void setOptions();
    MetadataNode findVlr(MetadataNode node, const std::string& recordId,
        const std::string& userId);
    void setVLRsFromMetadata(LasHeader& header, MetadataNode metaNode,
        Options const& opts);
    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented

    template<typename T>
    T getMetadataOption(pdal::Options const& options,
        pdal::MetadataNode const& metaNode, std::string const& name,
        T default_value) const
    {
        boost::optional<std::string> candidate =
            options.getMetadataOption<std::string>(name);

        // If this field isn't a metadata option, just return the plain option
        // value or the default value.
        if (!candidate)
            return options.getValueOrDefault<T>(name, default_value);
            
        // If the metadata option for this field is "FORWARD", return the
        // value from the metadata or the default value.
        if (boost::algorithm::iequals(*candidate, "FORWARD"))
        {
            auto pred = [name](MetadataNode n)
            {
                return n.name() == name;
            };
            MetadataNode m = metaNode.findChild(pred);
            if (!m.empty())
            {
                T t;
                std::istringstream iss(m.value());
                iss >> t;
                return t;
            }
            return default_value;
        }
        // Just return the value from the stored metadata option.
        return boost::lexical_cast<T>(*candidate);
    }    
};

} // namespace las
} // namespace drivers
} // namespace pdal

