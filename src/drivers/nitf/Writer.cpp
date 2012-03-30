/******************************************************************************
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

#include <pdal/drivers/nitf/Writer.hpp>
#include <pdal/drivers/las/Writer.hpp>

#include <pdal/PointBuffer.hpp>


// NOTES
//
// is it legal to write a LAZ file?
// syntactically, how do we name all the LAS writer options that we will pass to the las writer?
//

namespace pdal { namespace drivers { namespace nitf {


Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
    , m_streamManager(options.getOption("filename").getValue<std::string>())
    , m_lasWriter(NULL)
{
    ctor();
    return;
}


Writer::Writer(Stage& prevStage, std::ostream* ostream)
    : pdal::Writer(prevStage, Options::none())
    , m_streamManager(ostream)
    , m_lasWriter(NULL)
{
    ctor();
    return;
}


void Writer::ctor()
{
    m_streamManager.open();
    std::ostream& ostr = m_streamManager.ostream();
    m_lasWriter = new pdal::drivers::las::Writer(getPrevStage(), &ostr);
    return;
}


Writer::~Writer()
{
    m_streamManager.close();

    delete m_lasWriter;

    return;
}


void Writer::initialize()
{
    //pdal::Writer::initialize();

    m_lasWriter->initialize();

    return;
}


const Options Writer::getDefaultOptions() const
{
    // BUG: fix this
    Options options;
    return options;
}


void Writer::writeBegin(boost::uint64_t /*targetNumPointsToWrite*/)
{
    // shouldn't be called, since we override write()
    throw pdal::internal_error("NITF writer protected function was called");
}


boost::uint32_t Writer::writeBuffer(const PointBuffer& /*buffer*/)
{
    // shouldn't be called, since we override write()
    throw pdal::internal_error("NITF writer protected function was called");
}


void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
    // shouldn't be called, since we override write()
    throw pdal::internal_error("NITF writer protected function was called");
}


boost::uint64_t Writer::write(boost::uint64_t targetNumPointsToWrite)
{
    return m_lasWriter->write(targetNumPointsToWrite);
}


boost::property_tree::ptree Writer::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Writer::toPTree();

    // add stuff here specific to this stage type

    return tree;
}

} } } // namespaces
