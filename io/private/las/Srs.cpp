/******************************************************************************
* Copyright (c) 2021, Hobu Inc. (hobu@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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

#include "Geotiff.hpp"
#include "Srs.hpp"
#include "Utils.hpp"

namespace pdal
{
namespace las
{

void Srs::init(const VlrList& vlrs, std::vector<SrsType> srsOrder, bool useWkt, LogPtr log)
{
    bool specifiedOrder = srsOrder.size();

    m_srs = SpatialReference();
    m_geotiffString.clear();

    if (srsOrder.empty())
    {
        if (useWkt)
            srsOrder = { SrsType::Wkt2, SrsType::Proj, SrsType::Wkt1 };
        else
            srsOrder = { SrsType::Wkt2, SrsType::Proj, SrsType::Geotiff };
    }

    const Vlr *wkt2 = findVlr(TransformUserId, LASFWkt2recordId, vlrs);
    const Vlr *proj = findVlr(PdalUserId, PdalProjJsonRecordId, vlrs);
    const Vlr *gtiff = findVlr(TransformUserId, GeotiffDirectoryRecordId, vlrs);
    const Vlr *wkt1 = findVlr(TransformUserId, WktRecordId, vlrs);
    if (!wkt1)
        wkt1 = findVlr(LiblasUserId, WktRecordId, vlrs);

    if (wkt1 && gtiff && log)
        log->get(LogLevel::Debug) << "File contains both "
            "WKT and GeoTiff VLRs which is disallowed." << std::endl;

    try
    {
        for (const SrsType& type : srsOrder)
        {
            if (type == SrsType::Wkt2 && wkt2)
            {
                extractWkt(wkt2);
                break;
            }
            else if (type == SrsType::Proj && proj)
            {
                extractWkt(proj);
                break;
            }
            else if (type == SrsType::Wkt1 && wkt1)
            {
                extractWkt(wkt1);
                break;
            }
            else if (type == SrsType::Geotiff && gtiff)
            {
                extractGeotiff(gtiff, vlrs, log);
                break;
            }
        }
    }
    catch (...)
    {
        if (log)
            log->get(LogLevel::Error) << "Could not create an SRS.\n";
    }
    if (specifiedOrder && m_srs.empty() && log)
        log->get(LogLevel::Warning) << "'srs_vlr_order' specified but no valid VLR was found.";
}

void Srs::extractGeotiff(const Vlr *vlr, const VlrList& vlrs, LogPtr log)
{
    if (!vlr)
        return;

    const char *data = vlr->data();
    size_t dataLen = vlr->dataSize();

    std::vector<char> directoryRec(data, data + dataLen);

    vlr = findVlr(TransformUserId, GeotiffDoublesRecordId, vlrs);
    data = NULL;
    dataLen = 0;
    if (vlr && !vlr->empty())
    {
        data = vlr->data();
        dataLen = vlr->dataSize();
    }
    std::vector<char> doublesRec(data, data + dataLen);

    vlr = findVlr(TransformUserId, GeotiffAsciiRecordId, vlrs);
    data = NULL;
    dataLen = 0;
    if (vlr && !vlr->empty())
    {
        data = vlr->data();
        dataLen = vlr->dataSize();
    }
    std::vector<char> asciiRec(data, data + dataLen);

    SpatialReference srs;
    try
    {
        GeotiffSrs geotiffSrs(directoryRec, doublesRec, asciiRec, log);
        m_geotiffString = geotiffSrs.gtiffPrintString();
        if (log)
            log->get(LogLevel::Debug3) << m_geotiffString << std::endl;
        m_srs = geotiffSrs.srs();
    }
    catch (Geotiff::error& err)
    {
        if (log)
            log->get(LogLevel::Error) << "Could not create an SRS: " << err.what() << ".\n";
    }
}

void Srs::extractWkt(const Vlr *vlr)
{
    if (!vlr || vlr->empty())
        return;

    // There is supposed to be a NULL byte at the end of the data,
    // but sometimes there isn't because some people don't follow the
    // rules.  If there is a NULL byte, don't stick it in the
    // wkt string.
    size_t len = vlr->dataSize();
    const char *c = vlr->data() + len - 1;
    if (*c == 0)
        len--;
    std::string wkt(vlr->data(), len);
    // Strip any excess NULL bytes from the WKT.
    wkt.erase(std::find(wkt.begin(), wkt.end(), '\0'), wkt.end());

    m_srs = SpatialReference(wkt);
}


const SpatialReference& Srs::get() const
{
    return m_srs;
}

std::string Srs::geotiffString() const
{
    return m_geotiffString;
}

} // namespace las
} // namespace pdal
