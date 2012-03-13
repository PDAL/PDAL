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

#include <pdal/pdal_internal.hpp>
#ifdef PDAL_HAVE_PYTHON

#include <boost/test/unit_test.hpp>

#include <pdal/plang/PythonSupport.hpp>


BOOST_AUTO_TEST_SUITE(PythonTest)

BOOST_AUTO_TEST_CASE(PythonTest_basic)
{
    //pdal::plang::PythonEnvironment::startup();

    const char* program =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  #print 'hi'\n"
        "  return\n"
        ;

    pdal::plang::PythonMethodX meth(program);
    meth.compile();
    meth.execute();

    //pdal::plang::PythonEnvironment::shutdown();

    return;
}


BOOST_AUTO_TEST_CASE(PythonTest_runtime_error)
{
    const char* program =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  z['s'] = 9\n"
        "  return\n"
        ;

    pdal::plang::PythonMethodX meth(program);
    meth.compile();
    
    BOOST_REQUIRE_THROW(meth.execute(), pdal::python_error);

    return;
}


BOOST_AUTO_TEST_CASE(PythonTest_compile_error)
{
    const char* program =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "return\n"
        ;

    pdal::plang::PythonMethodX meth(program);

    BOOST_REQUIRE_THROW(meth.compile(), pdal::python_error);

    return;
}


BOOST_AUTO_TEST_CASE(PythonTest_ins)
{
    double data[5] = {1.0, 2.0, 3.0, 4.0, 5.0};

    const char* program =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  #print ins['X']\n"
        "  X = ins['X']\n"
        "  #print X\n"
        "  return\n"
        ;

    pdal::plang::PythonMethodX meth(program);
    meth.compile();
    
    meth.insertArgument("X", (boost::uint8_t*)data, 5, 8, pdal::dimension::Float, 8);

    meth.execute();

    return;
}


BOOST_AUTO_TEST_CASE(PythonTest_outs)
{
    const char* program =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  #print outs['X']\n"
        "  X = np.ones(5)\n"
        "  #print X\n"
        "  outs['X'] = X\n"
        "  #print outs['X']\n"
        "  return\n"
        ;

    pdal::plang::PythonMethodX meth(program);
    meth.compile();
    
    meth.execute();
    
    BOOST_CHECK(meth.hasOutputVariable("X"));

    double data[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    meth.extractResult("X", (boost::uint8_t*)data, 5, 8, pdal::dimension::Float, 8);

    BOOST_CHECK(data[0] == 1.0);
    BOOST_CHECK(data[1] == 1.0);
    BOOST_CHECK(data[2] == 1.0);
    BOOST_CHECK(data[3] == 1.0);
    BOOST_CHECK(data[4] == 1.0);

    return;
}

BOOST_AUTO_TEST_SUITE_END()

#endif
