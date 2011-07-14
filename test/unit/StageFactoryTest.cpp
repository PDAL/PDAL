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

#include "Support.hpp"

#include <pdal/StageFactory.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Filter.hpp>
#include <pdal/Writer.hpp>
#include <pdal/Options.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(StageFactoryTest)


BOOST_AUTO_TEST_CASE(test1)
{
    StageFactory factory;

    boost::shared_ptr<Reader> ptrR;
    boost::shared_ptr<Filter> ptrF;
    boost::shared_ptr<Writer> ptrW;
    
    const Options optsR("filename", Support::datapath("1.2-with-color.las"), "file to read from");
    ptrR = factory.createReader("drivers.las.reader", optsR);
    BOOST_CHECK(ptrR->getName() == "drivers.las.reader");

    const Options optsF("bounds", Bounds<double>(0,0,0,1,1,1), "crop bounds");
    ptrF = factory.createFilter("filters.crop", *ptrR, optsF);
    BOOST_CHECK(ptrF->getName() == "filters.crop");

    const Options optsW("filename", "temp.las", "file to write to");
    ptrW = factory.createWriter("drivers.las.writer", *ptrF, optsW);
    BOOST_CHECK(ptrW->getName() == "drivers.las.writer");

    return;
}


BOOST_AUTO_TEST_SUITE_END()
