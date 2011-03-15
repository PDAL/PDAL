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

#include <cassert>
#include <libpc/filters/MosaicFilter.hpp>
#include <libpc/exceptions.hpp>

namespace libpc
{


MosaicFilter::MosaicFilter(Stage& prevStage, std::vector<Stage*> prevStages)
    : Filter(prevStage)
{
    m_prevStages.push_back(&prevStage);

    const Header& prevHeader =  m_prevStage.getHeader();

    boost::uint64_t totalPoints = 0;

    Bounds<double> bigbox(prevHeader.getBounds());

    for (size_t i=0; i<prevStages.size(); i++)
    {
        Stage* stage = prevStages[i];
        if (stage==NULL)
        {
            throw libpc_error("bad stage passed to MosaicFilter");
        }
        const Header& header = stage->getHeader();
        if (prevHeader.getSchema() != header.getSchema())
        {
            throw libpc_error("impedance mismatch in MosaicFilter");
        }

        bigbox.grow(header.getBounds());
        totalPoints += header.getNumPoints();
        m_prevStages.push_back(stage);
    }

    Header& thisHeader = getHeader();
    thisHeader.setBounds(bigbox);
    thisHeader.setNumPoints(totalPoints);

    return;
}


const std::string& MosaicFilter::getName() const
{
    static std::string name("Mosaic Filter");
    return name;
}


boost::uint32_t MosaicFilter::readBuffer(PointData& destData)
{
    // BUG: We know that the two prev stage schemas are compatible, 
    // but we can't be sure the have the same bitfield layouts as 
    // the buffer we've been given.  We could handle it manually if 
    // they differ, but that would be a pain for now.  (This affects
    // all filters, I guess.)

    // BUG: this doesn't account for isValid()

    boost::uint32_t totalNumPointsToRead = destData.getNumPoints();
    boost::uint32_t totalNumPointsRead = 0;

    boost::uint64_t currentPointIndex = getCurrentPointIndex();

    int destPointIndex = 0;
    boost::uint64_t stageStartIndex = 0;

    // for each stage, we read as many points as we can
    for (size_t i=0; i<m_prevStages.size(); i++)
    {
        Stage& stage = *(m_prevStages[i]);

        const boost::uint64_t stageStopIndex = stageStartIndex + stage.getNumPoints();

        if (currentPointIndex < stageStopIndex)
        {
            // we need to read some points from this stage

            boost::uint32_t pointsAvail = (boost::uint32_t)(stageStopIndex - currentPointIndex);
            boost::uint32_t pointsToGet = std::min(pointsAvail, totalNumPointsToRead);

            PointData srcData(destData.getSchemaLayout(), pointsToGet);
            boost::uint32_t pointsGotten = stage.read(srcData);

            for (boost::uint32_t idx=0; idx<pointsGotten; idx++)
            {
                if (srcData.isValid(idx))
                {
                    destData.copyPointFast(destPointIndex, idx, srcData);
                    destData.setValid(destPointIndex, true);
                }
                else
                {
                    destData.setValid(destPointIndex, false);
                }
                destPointIndex++;
            }

            totalNumPointsRead += pointsGotten;
            currentPointIndex += pointsGotten;
        }

        stageStartIndex += stage.getNumPoints();

        if (totalNumPointsRead == totalNumPointsToRead) break;
    }

    return totalNumPointsRead;
}


} // namespace libpc
