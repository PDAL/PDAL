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

#include <pdal/plang/Invocation.hpp>

using namespace pdal;
using namespace pdal::plang;

BOOST_AUTO_TEST_SUITE(PLangTest)

BOOST_AUTO_TEST_CASE(PLangTest_basic)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  #print 'hi'\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();
    meth.execute();
}


//---------------------------------------------------------------------------
//
// Error tests
//
//---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(PLangTest_compile_error)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);

    BOOST_REQUIRE_THROW(meth.compile(), python_error);
}


BOOST_AUTO_TEST_CASE(PLangTest_runtime_error)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  z['s'] = 9\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();

    BOOST_REQUIRE_THROW(meth.execute(), pdal::python_error);
}


BOOST_AUTO_TEST_CASE(PLangTest_toofewinputs)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins):\n"
        "  #print 'foo'\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();

    BOOST_REQUIRE_THROW(meth.execute(), pdal::python_error);
}


BOOST_AUTO_TEST_CASE(PLangTest_toomanyinputs)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs,mids):\n"
        "  #print 'foo'\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();

    BOOST_REQUIRE_THROW(meth.execute(), pdal::python_error);
}


BOOST_AUTO_TEST_CASE(PLangTest_returnvoid)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs,mids):\n"
        "  #print 'foo'\n"
        "  return\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();

    BOOST_REQUIRE_THROW(meth.execute(), pdal::python_error);
}


BOOST_AUTO_TEST_CASE(PLangTest_returnint)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs,mids):\n"
        "  #print 'foo'\n"
        "  return 7\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();

    BOOST_REQUIRE_THROW(meth.execute(), pdal::python_error);
}


//---------------------------------------------------------------------------
//
// PARAM tests
//
//---------------------------------------------------------------------------


BOOST_AUTO_TEST_CASE(PLangTest_ins)
{
    double data[5] = {1.0, 2.0, 3.0, 4.0, 5.0};

    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  #print ins['X']\n"
        "  X = ins['X']\n"
        "  #print X\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();
    meth.insertArgument("X", (boost::uint8_t*)data, Dimension::Type::Double, 5);
    meth.execute();
}


BOOST_AUTO_TEST_CASE(PLangTest_outs)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  #print outs['X']\n"
        "  X = np.ones(5)\n"
        "  #print X\n"
        "  outs['X'] = X\n"
        "  #print outs['X']\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();
    meth.execute();
    BOOST_CHECK(meth.hasOutputVariable("X"));
    void *output = meth.extractResult("X", Dimension::Type::Double);

    double *d = (double *)output;
    BOOST_CHECK_CLOSE(*d++, 1.0, 0.00001);
    BOOST_CHECK_CLOSE(*d++, 1.0, 0.00001);
    BOOST_CHECK_CLOSE(*d++, 1.0, 0.00001);
    BOOST_CHECK_CLOSE(*d++, 1.0, 0.00001);
    BOOST_CHECK_CLOSE(*d++, 1.0, 0.00001);
}


BOOST_AUTO_TEST_CASE(PLangTest_aliases)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  \n"
        "  #print ins['X']\n"
        "  #print ins['prefix.X']\n"
        "  \n"
        "  X = ins['X']\n"
        "  prefixX = ins['prefix.X']\n"
        "  \n"
        "  #print X\n"
        "  #print prefixX\n"
        "  \n"
        "  Y = X + prefixX\n"
        "  prefixY = Y\n"
        "  \n"
        "  #print Y\n"
        "  #print prefixY\n"
        "  \n"
        "  outs['Y'] = Y\n"
        "  outs['prefix.Y'] = prefixY\n"
        "  \n"
        "  #print outs['Y']\n"
        "  #print outs['prefix.Y']\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();

    {
        double data[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
        meth.insertArgument("X", (uint8_t*)data, Dimension::Type::Double, 5);
        meth.insertArgument("prefix.X", (uint8_t*)data,
            Dimension::Type::Double, 5);
    }
    meth.execute();

    {
        BOOST_CHECK(meth.hasOutputVariable("Y"));
        BOOST_CHECK(meth.hasOutputVariable("prefix.Y"));

        void *output = meth.extractResult("Y", Dimension::Type::Double);
        double *d = (double *)output;
        BOOST_CHECK_CLOSE(*d++, 2.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 4.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 6.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 8.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 10.0, 0.00001);

        output = meth.extractResult("prefix.Y", Dimension::Type::Double);
        d = (double *)output;
        BOOST_CHECK_CLOSE(*d++, 2.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 4.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 6.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 8.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 10.0, 0.00001);
    }

    {
        std::vector<std::string> names;
        meth.getOutputNames(names);
        BOOST_CHECK(names.size() == 2);
        BOOST_CHECK(names[0] == "Y");
        BOOST_CHECK(names[1] == "prefix.Y");
    }
}


BOOST_AUTO_TEST_CASE(PLangTest_returntrue)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();

    bool sts = meth.execute();
    BOOST_CHECK(sts);
}


BOOST_AUTO_TEST_CASE(PLangTest_returnfalse)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  return False\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();

    bool sts = meth.execute();
    BOOST_CHECK(!sts);
}


//---------------------------------------------------------------------------
//
// MISC tests
//
//---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(PLangTest_reentry)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  X = ins['X']\n"
        "  Y = X + 1.0\n"
        "  #print Y\n"
        "  outs['Y'] = Y\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();

    {
        double indata1[5] = {0.0, 1.0, 2.0, 3.0, 4.0};
        meth.insertArgument("X", (uint8_t*)indata1, Dimension::Type::Double, 5);
        meth.execute();
        void *output = meth.extractResult("Y", Dimension::Type::Double);

        double *d = (double *)output;
        BOOST_CHECK_CLOSE(*d++, 1.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 2.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 3.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 4.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 5.0, 0.00001);
    }

    {
        double indata2[5] = {10.0, 20.0, 30.0, 40.0, 50.0};
        meth.insertArgument("X", (uint8_t*)indata2, Dimension::Type::Double, 5);
        meth.execute();
        void *output = meth.extractResult("Y", Dimension::Type::Double);

        double *d = (double *)output;
        BOOST_CHECK_CLOSE(*d++, 11.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 21.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 31.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 41.0, 0.00001);
        BOOST_CHECK_CLOSE(*d++, 51.0, 0.00001);
    }
}

BOOST_AUTO_TEST_SUITE_END()

#endif
