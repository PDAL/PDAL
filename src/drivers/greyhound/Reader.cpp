/******************************************************************************
* Copyright (c) 2014, Connor Manning (connor@hobu.co)
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

#include <pdal/drivers/greyhound/Reader.hpp>

namespace pdal
{
namespace drivers
{
namespace greyhound
{

Options GreyhoundReader::getDefaultOptions()
{
    Options options;

    // TODO

    return options;
}

std::vector<Dimension> GreyhoundReader::getDefaultDimensions()
{
    std::vector<Dimension> output;

    // TODO

    return output;
}

StageSequentialIterator* GreyhoundReader::createSequentialIterator() const
{
    return new iterators::sequential::Iterator();
}

void GreyhoundReader::processOptions(const Options& options)
{
    // TODO Initialize connection info
}

void GreyhoundReader::buildSchema(Schema* schema)
{
    std::vector<Dimension> dims = getDefaultDimensions();
    for (auto d = dims.begin(); d != dims.end(); ++d)
        m_dims.push_back(schema->appendDimension(*d));
}

void GreyhoundReader::ready(PointContext ctx)
{
    // TODO
}

namespace iterators
{
sequential::Iterator::Iterator()
{

}

point_count_t sequential::Iterator::readImpl(
        PointBuffer& data,
        point_count_t count)
{
    // TODO
    std::cerr << "No sequential readImpl for stage/iterator!\n";
    return 0;
}

boost::uint64_t sequential::Iterator::skipImpl(
        const boost::uint64_t pointsToSkip)
{
    // TODO
    return 0;
}

boost::uint32_t sequential::Iterator::readBufferImpl(PointBuffer&)
{
    // TODO
    return 0;
}

bool sequential::Iterator::atEndImpl() const
{
    // TODO
    return true;
}

} // namespace iterators

} // namespace greyhound
} // namespace drivers
} // namespace pdal

