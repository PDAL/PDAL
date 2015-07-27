/****************************************************************************
* Copyright (c) 2012, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include "NitfWriter.hpp"

#include <pdal/pdal_macros.hpp>

#include <pdal/PointView.hpp>
#include <pdal/GlobalEnvironment.hpp>

#ifdef PDAL_COMPILER_GCC
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wredundant-decls"
#  pragma GCC diagnostic ignored "-Wextra"
#  pragma GCC diagnostic ignored "-Wcast-qual"
   // The following pragma doesn't actually work:
   //   https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61653
   //#  pragma GCC diagnostic ignored "-Wliteral-suffix"
#endif
#include <ogr_spatialref.h>
#include <cpl_conv.h>
#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wunused-private-field"
#endif

#define IMPORT_NITRO_API
#include <nitro/c++/import/nitf.hpp>
#include "tre_plugins.hpp"

#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic pop
#endif
#ifdef PDAL_COMPILER_GCC
#  pragma GCC diagnostic pop
#endif

// NOTES
//
// is it legal to write a LAZ file?
// syntactically, how do we name all the LAS writer options that we will pass to the las writer?
//

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.nitf",
    "NITF Writer",
    "http://pdal.io/stages/writers.nitf.html" );

CREATE_SHARED_PLUGIN(1, 0, NitfWriter, Writer, s_info)

std::string NitfWriter::getName() const { return s_info.name; }

BOX3D reprojectBoxToDD(const SpatialReference& reference, const BOX3D& box)
{
    if (reference.empty())
        return BOX3D();

    BOX3D output(box);

    OGRSpatialReferenceH current =
        OSRNewSpatialReference(reference.getWKT(SpatialReference::eCompoundOK,
            false).c_str());
    OGRSpatialReferenceH dd = OSRNewSpatialReference(0);

    OGRErr err = OSRSetFromUserInput(dd, const_cast<char *>("EPSG:4326"));
    if (err != OGRERR_NONE)
        throw std::invalid_argument("could not import coordinate system "
            "into OGRSpatialReference SetFromUserInput");

    OGRCoordinateTransformationH transform =
        OCTNewCoordinateTransformation(current, dd);

    int ret = OCTTransform(transform, 1,
        &output.minx, &output.miny, &output.minz);
    if (ret == 0)
    {
        std::ostringstream msg;
        msg << "Could not project point for reprojectBoxToDD::min" <<
            CPLGetLastErrorMsg() << ret;
        throw pdal_error(msg.str());
    }

    OCTTransform(transform, 1, &output.maxx, &output.maxy, &output.maxz);
    if (ret == 0)
    {
        std::ostringstream msg;
        msg << "Could not project point for reprojectBoxToDD::max" <<
            CPLGetLastErrorMsg() << ret;
        throw pdal_error(msg.str());
    }

    OCTDestroyCoordinateTransformation(transform);
    OSRDestroySpatialReference(current);
    OSRDestroySpatialReference(dd);

    return output;
}


NitfWriter::NitfWriter()
{
    register_tre_plugins();
}


void NitfWriter::processOptions(const Options& options)
{
    LasWriter::processOptions(options);
    m_cLevel = options.getValueOrDefault<std::string>("CLEVEL","03");
    m_sType = options.getValueOrDefault<std::string>("STYPE","BF01");
    m_oStationId = options.getValueOrDefault<std::string>("OSTAID","PDAL");
    m_fileTitle = options.getValueOrDefault<std::string>("FTITLE", "");
    m_fileClass = options.getValueOrDefault<std::string>("FSCLAS","U");
    m_origName = options.getValueOrDefault<std::string>("ONAME","");
    m_origPhone = options.getValueOrDefault<std::string>("OPHONE","");
    m_securityClass = options.getValueOrDefault<std::string>("FSCLAS","U");
    m_securityControlAndHandling =
        options.getValueOrDefault<std::string>("FSCTLH","");
    m_securityClassificationSystem =
        options.getValueOrDefault<std::string>("FSCLSY","");
    m_imgSecurityClass = options.getValueOrDefault<std::string>("FSCLAS","U");
    m_imgDate = getOptions().getValueOrDefault<std::string>("IDATIM", "");
    m_imgIdentifier2 = getOptions().getValueOrDefault<std::string>("IID2", "");
    m_sic = getOptions().getValueOrDefault<std::string>("FSCLTX", "");
    try
    {
        m_aimidb = getOptions().getOption("AIMIDB");
    }
    catch (Option::not_found)
    {}
    try
    {
        m_acftb = getOptions().getOption("ACFTB");
    }
    catch (Option::not_found)
    {}
}


void NitfWriter::writeView(const PointViewPtr view)
{
    view->calculateBounds(m_bounds);

    LasWriter::writeView(view);
}


void NitfWriter::readyFile(const std::string& filename)
{
    m_error.setFilename(filename);
    m_nitfFilename = filename;
    prepOutput(&m_oss);
}


void NitfWriter::doneFile()
{
    finishOutput();

    try
    {
        ::nitf::Record record(NITF_VER_21);
        ::nitf::FileHeader header = record.getHeader();
        header.getFileHeader().set("NITF");
        header.getComplianceLevel().set(m_cLevel);
        header.getSystemType().set(m_sType);
        header.getOriginStationID().set(m_oStationId);
        if (m_fileTitle.empty())
            m_fileTitle = m_nitfFilename;
        header.getFileTitle().set(m_fileTitle);
        header.getClassification().set(m_fileClass);
        header.getMessageCopyNum().set("00000");
        header.getMessageNumCopies().set("00000");
        header.getEncrypted().set("0");
        header.getBackgroundColor().setRawData(const_cast<char*>("000"), 3);
        header.getOriginatorName().set(m_origName);
        header.getOriginatorPhone().set(m_origPhone);
        header.getSecurityGroup().getClassificationSystem().set(
            m_securityClassificationSystem);
        header.getSecurityGroup().getControlAndHandling().set(
            m_securityControlAndHandling);
        header.getSecurityGroup().getClassificationText().set(m_sic);

        ::nitf::DESegment des = record.newDataExtensionSegment();

        des.getSubheader().getFilePartType().set("DE");
        des.getSubheader().getTypeID().set("LIDARA DES");
        des.getSubheader().getVersion().set("01");
        des.getSubheader().getSecurityClass().set(m_securityClass);
        ::nitf::FileSecurity security = record.getHeader().getSecurityGroup();
        des.getSubheader().setSecurityGroup(security.clone());

        ::nitf::TRE usrHdr("LIDARA DES", "raw_data");
        usrHdr.setField("raw_data", "not");
        ::nitf::Field fld = usrHdr.getField("raw_data");
        fld.setType(::nitf::Field::BINARY);

        std::streambuf *buf = m_oss.rdbuf();
        long size = buf->pubseekoff(0, m_oss.end);
        buf->pubseekoff(0, m_oss.beg);

        std::vector<char> bytes(size);
        buf->sgetn(bytes.data(), size);
        m_oss.clear();

        des.getSubheader().setSubheaderFields(usrHdr);

        ::nitf::ImageSegment image = record.newImageSegment();
        ::nitf::ImageSubheader subheader = image.getSubheader();

        BOX3D bounds =  reprojectBoxToDD(m_srs, m_bounds);

        //NITF decimal degree values for corner coordinates only has a
        // precision of 3 after the decimal. This may cause an invalid
        // polygon due to rounding errors with a small tile. Therefore
        // instead of rounding min values will use the floor value and
        // max values will use the ceiling values.
        bounds.minx = (floor(bounds.minx * 1000)) / 1000.0;
        bounds.miny = (floor(bounds.miny * 1000)) / 1000.0;
        bounds.maxx = (ceil(bounds.maxx * 1000)) / 1000.0;
        bounds.maxy = (ceil(bounds.maxy * 1000)) / 1000.0;

        double corners[4][2];
        corners[0][0] = bounds.maxy;
        corners[0][1] = bounds.minx;
        corners[1][0] = bounds.maxy;
        corners[1][1] = bounds.maxx;
        corners[2][0] = bounds.miny;
        corners[2][1] = bounds.maxx;
        corners[3][0] = bounds.miny;
        corners[3][1] = bounds.minx;
        subheader.setCornersFromLatLons(NRT_CORNERS_DECIMAL, corners);

        subheader.getImageSecurityClass().set(m_imgSecurityClass);
        subheader.setSecurityGroup(security.clone());
        if (m_imgDate.size())
            subheader.getImageDateAndTime().set(m_imgDate);

        ::nitf::BandInfo info;
        ::nitf::LookupTable lt(0,0);
        info.init("G",    /* The band representation, Nth band */
                  " ",      /* The band subcategory */
                  "N",      /* The band filter condition */
                  "   ",    /* The band standard image filter code */
                  0,        /* The number of look-up tables */
                  0,        /* The number of entries/LUT */
                  lt);     /* The look-up tables */

        std::vector< ::nitf::BandInfo> bands;
        bands.push_back(info);
        subheader.setPixelInformation(
            "INT",      /* Pixel value type */
            8,         /* Number of bits/pixel */
            8,         /* Actual number of bits/pixel */
            "R",       /* Pixel justification */
            "NODISPLY",     /* Image representation */
            "VIS",     /* Image category */
            1,         /* Number of bands */
            bands);

        subheader.setBlocking(
            8,   /*!< The number of rows */
            8,  /*!< The number of columns */
            8, /*!< The number of rows/block */
            8,  /*!< The number of columns/block */
            "P");                /*!< Image mode */

        //Image Header fields to set
        subheader.getImageId().set("None");
        subheader.getImageTitle().set(m_imgIdentifier2);

        // 64 char string
        std::string zeros(64, '0');

        std::unique_ptr< ::nitf::BandSource> band(new ::nitf::MemorySource(
            const_cast<char*>(zeros.c_str()),
            zeros.size() /* memory size */,
            0 /* starting offset */,
            1 /* bytes per pixel */,
            0 /*skip*/));
        ::nitf::ImageSource iSource;
        iSource.addBand(*band);

        //AIMIDB
        if (!m_aimidb.empty())
        {
            boost::optional<const Options&> options = m_aimidb.getOptions();
            if (options)
            {
                ::nitf::TRE tre("AIMIDB");
                std::vector<Option> opts = options->getOptions();
                for (auto i = opts.begin(); i != opts.end(); ++i)
                {
                    tre.setField(i->getName(), i->getValue<std::string>());
                }
                subheader.getExtendedSection().appendTRE(tre);
            }
        }

        //ACFTB
        if (!m_acftb.empty())
        {
            boost::optional<const Options&> options = m_acftb.getOptions();
            if (options)
            {
                ::nitf::TRE tre("ACFTB");
                std::vector<Option> opts = options->getOptions();
                for (auto i = opts.begin(); i != opts.end(); ++i)
                {
                    tre.setField(i->getName(), i->getValue<std::string>());
                }
                subheader.getExtendedSection().appendTRE(tre);
            }
        }

        ::nitf::Writer writer;
        ::nitf::IOHandle output_io(m_nitfFilename.c_str(),
            NITF_ACCESS_WRITEONLY, NITF_CREATE);
        writer.prepare(output_io, record);

        ::nitf::SegmentWriter sWriter = writer.newDEWriter(0);

        ::nitf::SegmentMemorySource sSource(bytes.data(), size, 0, 0, false);
        sWriter.attachSource(sSource);

        ::nitf::ImageWriter iWriter = writer.newImageWriter(0);
        iWriter.attachSource(iSource);

        writer.write();
        output_io.close();
    }
    catch (except::Throwable & t)
    {
        std::ostringstream oss;
        // std::cout << t.getTrace();
        throw pdal_error(t.getMessage());
    }
}

} // namespaces
