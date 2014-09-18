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

#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/las/Support.hpp>
#include <pdal/drivers/las/SummaryData.hpp>


namespace pdal
{
namespace drivers
{
namespace las
{

void Support::rewriteHeader(std::ostream& stream, const SummaryData& data)
{
    // move from header start to "number of point records" field
    stream.seekp(107, std::ios_base::cur);

    //This is legacy for 1.4 -- only valid on read for 1.3 and before.
    const int MAX_RETURNS = 5; 
    {
        boost::uint8_t buf[256];
        boost::uint8_t* p = buf;

        Utils::write_field<uint32_t>(p, data.getTotalNumPoints());
        for (int i = 1; i <= MAX_RETURNS; i++)
            Utils::write_field<uint32_t>(p, data.getReturnCount(i));

        // Write the data to the stream. 
        Utils::write_n(stream, buf, 4 + 4 * MAX_RETURNS);
    }

    // skip over scale/offset fields
    stream.seekp(8 * 6, std::ios_base::cur);

    {
        boost::uint8_t buf[256];
        boost::uint8_t* p = buf;

        pdal::Bounds<double> bounds = data.getBounds();
        double minX = bounds.getMinimum(0);
        double minY = bounds.getMinimum(1);
        double minZ = bounds.getMinimum(2);
        double maxX = bounds.getMaximum(0);
        double maxY = bounds.getMaximum(1);
        double maxZ = bounds.getMaximum(2);
        Utils::write_field<double>(p, maxX);
        Utils::write_field<double>(p, minX);
        Utils::write_field<double>(p, maxY);
        Utils::write_field<double>(p, minY);
        Utils::write_field<double>(p, maxZ);
        Utils::write_field<double>(p, minZ);
        Utils::write_n(stream, buf, 6*8);
    }

    stream.seekp(0, std::ios_base::end);

    return;
}


}
}
} // namespaces
