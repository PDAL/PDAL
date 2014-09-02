/******************************************************************************
* Copyright (c) 2014, Peter J. Gadomski (pete.gadomski@gmail.com)
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

#include <pdal/drivers/rxp/RxpReader.hpp>


namespace pdal
{
namespace drivers
{
namespace rxp
{


std::string extractRivlibURI(const Options& options)
{
    if (options.hasOption("filename"))
    {
        if (options.hasOption("rdtp"))
        {
            throw option_not_found("Cannot create URI when both filename and rdtp are provided");
        }
        return "file:" + options.getValueOrThrow<std::string>("filename");
    }
    else
    {
        return "rdtp://" + options.getValueOrThrow<std::string>("rdtp");
    }
}


Dimension::Id::Enum getTimeDimensionId(bool syncToPps)
{
    return syncToPps ? Dimension::Id::GpsTime : Dimension::Id::InternalTime;
}


Dimension::IdList getRxpDimensions(bool syncToPps)
{
    using namespace Dimension;
    Dimension::IdList ids;

    ids.push_back(Id::X);
    ids.push_back(Id::Y);
    ids.push_back(Id::Z);
    ids.push_back(Id::EchoRange);
    ids.push_back(getTimeDimensionId(syncToPps));
    ids.push_back(Id::Amplitude);
    ids.push_back(Id::Reflectance);
    ids.push_back(Id::Deviation);
    ids.push_back(Id::BackgroundRadiation);
    ids.push_back(Id::IsPpsLocked);
    ids.push_back(Id::ReturnNumber);
    ids.push_back(Id::NumberOfReturns);

    return ids;
}


void registerRxpDimsOnContext(PointContext ctx, bool syncToPps)
{
    ctx.registerDims(getRxpDimensions(syncToPps));
}


Options RxpReader::getDefaultOptions()
{
    Option isSyncToPps("sync_to_pps", DEFAULT_SYNC_TO_PPS, "");
    Options options(isSyncToPps);
    return options;
}


void RxpReader::processOptions(const Options& options)
{
    m_uri = extractRivlibURI(options);
    m_syncToPps = options.getValueOrDefault<bool>("sync_to_pps",
                                                  DEFAULT_SYNC_TO_PPS);
}


void RxpReader::addDimensions(PointContext ctx)
{
    registerRxpDimsOnContext(ctx, m_syncToPps);
}


pdal::StageSequentialIterator* RxpReader::createSequentialIterator() const
{
    return new iterators::sequential::RxpSeqIterator(m_uri, m_syncToPps);
}


namespace iterators
{
namespace sequential
{


RxpSeqIterator::RxpSeqIterator(const std::string uri,
                               bool syncToPps)
    : pdal::ReaderSequentialIterator()
    , scanlib::pointcloud(syncToPps)
    , m_syncToPps(syncToPps)
    , m_rc(scanlib::basic_rconnection::create(uri))
    , m_dec(m_rc)
    , m_savedPoints(NULL)
    , m_savedPointsIdx(0)
{}


RxpSeqIterator::~RxpSeqIterator()
{
    m_rc->close();
    return;
}


point_count_t RxpSeqIterator::readImpl(PointBuffer& data, point_count_t numPoints)
{
    point_count_t numRead = 0;
    if (m_savedPoints && m_savedPointsIdx < m_savedPoints->size())
    {
        numRead += appendSavedPoints(data, numPoints);
        if (numRead == numPoints) return numRead;
        assert(m_savedPoints->size() == m_savedPointsIdx);
    }

    m_savedPoints.reset(new PointBuffer(data.context()));
    m_savedPointsIdx = 0;

    for (m_dec.get(m_rxpbuf); !m_dec.eoi(); m_dec.get(m_rxpbuf))
    {
        dispatch(m_rxpbuf.begin(), m_rxpbuf.end());
        if (numRead + m_savedPoints->size() >= numPoints) break;
    }

    numRead += appendSavedPoints(data, numPoints - numRead);
    return numRead;
}


point_count_t RxpSeqIterator::appendSavedPoints(PointBuffer& data, point_count_t numPoints)
{
    point_count_t numRead = 0;
    while (numRead < numPoints && m_savedPointsIdx < m_savedPoints->size())
    {
        data.appendPoint(*m_savedPoints, m_savedPointsIdx++);
        numRead++;
    }
    return numRead;
}


void RxpSeqIterator::on_echo_transformed(echo_type echo)
{
    if (!(scanlib::pointcloud::single == echo || scanlib::pointcloud::last == echo))
    {
        // Come back later, when we've got all the echos
        return;
    }

    using namespace Dimension;

    boost::uint32_t idx = m_savedPoints->size();
    typedef std::vector<scanlib::target> target_vector;
    for (target_vector::size_type i = 0; i != targets.size(); i++)
    {
        m_savedPoints->setField(Id::X, idx, targets[i].vertex[0]);
        m_savedPoints->setField(Id::Y, idx, targets[i].vertex[1]);
        m_savedPoints->setField(Id::Z, idx, targets[i].vertex[2]);
        m_savedPoints->setField(getTimeDimensionId(m_syncToPps), idx, targets[i].time);
        m_savedPoints->setField(Id::EchoRange, idx, targets[i].echo_range);
        m_savedPoints->setField(Id::Amplitude, idx, targets[i].amplitude);
        m_savedPoints->setField(Id::Reflectance, idx, targets[i].reflectance);
        m_savedPoints->setField(Id::Deviation, idx, targets[i].deviation);
        m_savedPoints->setField(Id::BackgroundRadiation, idx, targets[i].background_radiation);
        m_savedPoints->setField(Id::IsPpsLocked, idx, targets[i].is_pps_locked);
        m_savedPoints->setField(Id::ReturnNumber, idx, i + 1);
        m_savedPoints->setField(Id::NumberOfReturns, idx, targets.size());
        ++idx;
    }
}


bool RxpSeqIterator::atEndImpl() const
{
    return m_dec.eoi();
}


point_count_t RxpSeqIterator::skipImpl(uint64_t numPoints)
{
    PointContext ctx;
    registerRxpDimsOnContext(ctx, m_syncToPps);
    PointBuffer wastebin(ctx);
    return read(wastebin, numPoints);
}


}
} // namespace iterators::sequential


}
}
} // namespace pdal::drivers::rxp
