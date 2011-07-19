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

#include <pdal/MultiFilter.hpp>
#include <pdal/exceptions.hpp>

namespace pdal
{


MultiFilter::MultiFilter(const std::vector<const DataStagePtr> prevStages, const Options& options)
    : DataStage(prevStages, options)
{
    if (prevStages.size() == 0)
    {
        throw pdal_error("empty stage list passed to MosaicFilter");
    }

    {
        setCoreProperties(prevStages[0]);  // BUG: clearly insufficient
    }

    boost::uint64_t totalPoints = 0;

    Bounds<double> bigbox(prevStages[0]->getBounds());

    for (size_t i=0; i<prevStages.size(); i++)
    {
        const DataStagePtr stage = prevStages[i];
        if (prevStages[0]->getSchema() != this->getSchema())
        {
            throw pdal_error("impedance mismatch in MosaicFilter");
        }

        bigbox.grow(this->getBounds());
        totalPoints += this->getNumPoints();
    }

    setBounds(bigbox);
    setNumPoints(totalPoints);

    return;
}



} // namespace pdal
