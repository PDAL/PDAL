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

#include <pdal/pdal_internal.hpp>
#ifdef PDAL_COMPILER_MSVC
#pragma warning(disable: 4806)  // until we fix PointBuffer.hpp
#endif

#include <pdal/drivers/nitf/Writer.hpp>
#include <pdal/drivers/las/Writer.hpp>

#include <pdal/PointBuffer.hpp>

#include "mpg_nitflib.hpp"


// NOTES
//
// is it legal to write a LAZ file?
// syntactically, how do we name all the LAS writer options that we will pass to the las writer?
//

namespace pdal
{
namespace drivers
{
namespace nitf
{

Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::drivers::las::Writer(prevStage, options)
{
    ctor();
    return;
}


Writer::Writer(Stage& prevStage, std::ostream* ostream)
    : pdal::drivers::las::Writer(prevStage, ostream)
{
    ctor();
    return;
}


void Writer::ctor()
{
    return;
}


Writer::~Writer()
{
    return;
}


void Writer::initialize()
{
    // call super class
    pdal::drivers::las::Writer::initialize();

    return;
}


const Options Writer::getDefaultOptions() const
{
    // BUG: fix this
    Options options;
    return options;
}


void Writer::writeBegin(boost::uint64_t targetNumPointsToWrite)
{
    m_streamManager.ostream().write(s_nitfHeader().c_str(), s_nitfHeader().size());

    // call super class
    pdal::drivers::las::Writer::writeBegin(targetNumPointsToWrite);

    mpg::NITFCreate("x", 1, 1, 1, 1, NULL, NULL);
}


void Writer::writeBufferBegin(PointBuffer const& buffer)
{
    // call super class
    pdal::drivers::las::Writer::writeBufferBegin(buffer);
}


boost::uint32_t Writer::writeBuffer(const PointBuffer& buffer)
{
    // call super class
    return pdal::drivers::las::Writer::writeBuffer(buffer);
}


void Writer::writeBufferEnd(PointBuffer const& buffer)
{
    // call super class
    pdal::drivers::las::Writer::writeBufferEnd(buffer);
}


void Writer::writeEnd(boost::uint64_t actualNumPointsWritten)
{
    // call super class
    pdal::drivers::las::Writer::writeEnd(actualNumPointsWritten);

    m_streamManager.ostream().write(s_nitfFooter().c_str(), s_nitfFooter().size());
}


boost::property_tree::ptree Writer::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Writer::toPTree();

    // add stuff here specific to this stage type

    return tree;
}

}
}
} // namespaces
