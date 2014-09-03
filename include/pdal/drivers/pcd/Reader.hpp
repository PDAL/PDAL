/******************************************************************************
* Copyright (c) 2014, Brad Chambers (brad.chambers@gmail.com)
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

#include <pdal/PointBuffer.hpp>
#include <pdal/Reader.hpp>
#include <pdal/ReaderIterator.hpp>
#include <pdal/drivers/pcd/Common.hpp>

namespace pdal
{
namespace drivers
{
namespace pcd
{

class PDAL_DLL PcdReader : public pdal::Reader
{
public:
    SET_STAGE_NAME ("drivers.pcd.reader", "PCD Reader")
    SET_STAGE_LINK ("http://pdal.io/stages/drivers.pcd.reader.html")
    SET_STAGE_ENABLED (true)

    PcdReader(const Options& options) : Reader(options) {};

    static Options getDefaultOptions();
    static Dimension::IdList getDefaultDimensions()
        { return fileDimensions(); };

    virtual StageSequentialIterator* createSequentialIterator() const;

private:
    std::string m_filename;
    point_count_t m_numPts;

    virtual void processOptions(const Options& options);
    virtual void addDimensions(PointContext ctx);
    virtual void ready(PointContext ctx);
};

namespace iterators
{
namespace sequential
{

class PDAL_DLL PcdSeqIterator : public pdal::ReaderSequentialIterator
{
public:
    PcdSeqIterator(const Dimension::IdList& dims, std::string filename) : m_dims(dims), m_filename(filename)
    {};

private:
    boost::uint64_t skipImpl(boost::uint64_t) {};
    point_count_t readImpl(PointBuffer& buf, std::string filename);
    bool atEndImpl() const {};

    virtual point_count_t readBufferImpl(PointBuffer& buf)
        { return readImpl(buf, m_filename); }

    Dimension::IdList m_dims;
    std::string m_filename;
};

} // sequential
} // iterators

}
}
} // namespaces

