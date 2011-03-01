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

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include "libpc/FauxReader.hpp"
#include "libpc/../../src/drivers/liblas/writer.hpp"
#include "libpc/../../src/drivers/liblas/reader.hpp"

#include "support.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(LiblasWriterTest)

BOOST_AUTO_TEST_CASE(test_simple_las)
{
    // remove file from earlier run, if needed
    Utils::deleteFile("temp.las");

    std::istream* ifs = Utils::openFile("../../test/data/1.2-with-color.las");
    LiblasReader reader(*ifs);
    
    std::ostream* ofs = Utils::createFile("temp.las");

    {
        const boost::uint64_t numPoints = reader.getHeader().getNumPoints();

        // need to scope the writer, so that's it dtor can use the stream
        LiblasWriter writer(reader, *ofs);

        writer.setCompressed(false);
        writer.setDate(0, 0);
        writer.setPointFormat(3);
        writer.setSystemIdentifier("");
        writer.setGeneratingSoftware("TerraScan");

        size_t np = (size_t)numPoints;
        assert(numPoints == np); // BUG
        writer.write(np);
    }

    Utils::closeFile(ofs);
    Utils::closeFile(ifs);

    bool filesSame = compare_files("temp.las", "../../test/data/simple.las");
    BOOST_CHECK(filesSame);

    if (filesSame)
    {
        Utils::deleteFile("temp.las");
    }

    return;
}

BOOST_AUTO_TEST_CASE(test_simple_laz)
{
    // remove file from earlier run, if needed
    Utils::deleteFile("temp.las");

    std::istream* ifs = Utils::openFile("../../test/data/1.2-with-color.las");
    LiblasReader reader(*ifs);
    
    std::ostream* ofs = Utils::createFile("temp.laz");

    {
        const boost::uint64_t numPoints = reader.getHeader().getNumPoints();

        // need to scope the writer, so that's it dtor can use the stream
        LiblasWriter writer(reader, *ofs);

        writer.setCompressed(true);
        writer.setDate(0, 0);
        writer.setPointFormat(3);
        writer.setSystemIdentifier("");
        writer.setGeneratingSoftware("TerraScan");

        size_t np = (size_t)numPoints;
        assert(numPoints == np); // BUG
        writer.write(np);
    }

    Utils::closeFile(ofs);
    Utils::closeFile(ifs);

    bool filesSame = compare_files("temp.laz", "../../test/data/simple.laz");
    BOOST_CHECK(filesSame);

    if (filesSame)
    {
        Utils::deleteFile("temp.laz");
    }

    return;
}

BOOST_AUTO_TEST_SUITE_END()
