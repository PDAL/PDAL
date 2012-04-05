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


BOOST_AUTO_TEST_SUITE(PLangTest)

BOOST_AUTO_TEST_CASE(PLangTest_basic)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  #print 'hi'\n"
        "  return True\n"
        ;
    pdal::plang::Script script(source, "MyTest", "yow");

    pdal::plang::Invocation meth(script);
    meth.compile();
    meth.execute();

    return;
}


//---------------------------------------------------------------------------
//
// ERROR tests
//
//---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(PLangTest_compile_error)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "return True\n"
        ;
    pdal::plang::Script script(source, "MyTest", "yow");

    pdal::plang::Invocation meth(script);

    BOOST_REQUIRE_THROW(meth.compile(), pdal::python_error);

    return;
}


BOOST_AUTO_TEST_CASE(PLangTest_runtime_error)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  z['s'] = 9\n"
        "  return True\n"
        ;
    pdal::plang::Script script(source, "MyTest", "yow");

    pdal::plang::Invocation meth(script);
    meth.compile();
    
    BOOST_REQUIRE_THROW(meth.execute(), pdal::python_error);

    return;
}


BOOST_AUTO_TEST_CASE(PLangTest_toofewinputs)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins):\n"
        "  #print 'foo'\n"
        "  return True\n"
        ;
    pdal::plang::Script script(source, "MyTest", "yow");

    pdal::plang::Invocation meth(script);
    meth.compile();
    
    BOOST_REQUIRE_THROW(meth.execute(), pdal::python_error);
    
    return;
}


BOOST_AUTO_TEST_CASE(PLangTest_toomanyinputs)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs,mids):\n"
        "  #print 'foo'\n"
        "  return True\n"
        ;
    pdal::plang::Script script(source, "MyTest", "yow");

    pdal::plang::Invocation meth(script);
    meth.compile();
    
    BOOST_REQUIRE_THROW(meth.execute(), pdal::python_error);
    
    return;
}


BOOST_AUTO_TEST_CASE(PLangTest_returnvoid)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs,mids):\n"
        "  #print 'foo'\n"
        "  return\n"
        ;
    pdal::plang::Script script(source, "MyTest", "yow");

    pdal::plang::Invocation meth(script);
    meth.compile();
    
    BOOST_REQUIRE_THROW(meth.execute(), pdal::python_error);
    
    return;
}


BOOST_AUTO_TEST_CASE(PLangTest_returnint)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs,mids):\n"
        "  #print 'foo'\n"
        "  return 7\n"
        ;
    pdal::plang::Script script(source, "MyTest", "yow");

    pdal::plang::Invocation meth(script);
    meth.compile();
    
    BOOST_REQUIRE_THROW(meth.execute(), pdal::python_error);
    
    return;
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
    pdal::plang::Script script(source, "MyTest", "yow");

    pdal::plang::Invocation meth(script);
    meth.compile();
    
    meth.insertArgument("X", (boost::uint8_t*)data, 5, 8, pdal::dimension::Float, 8);

    meth.execute();

    return;
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
    pdal::plang::Script script(source, "MyTest", "yow");

    pdal::plang::Invocation meth(script);
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


BOOST_AUTO_TEST_CASE(PLangTest_aliases)
{
    double data[5] = {1.0, 2.0, 3.0, 4.0, 5.0};

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
    pdal::plang::Script script(source, "MyTest", "yow");

    pdal::plang::Invocation meth(script);
    meth.compile();

    {
        meth.insertArgument("X", (boost::uint8_t*)data, 5, 8, pdal::dimension::Float, 8);
        meth.insertArgument("prefix.X", (boost::uint8_t*)data, 5, 8, pdal::dimension::Float, 8);
    }

    meth.execute();

    {
        BOOST_CHECK(meth.hasOutputVariable("Y"));
        BOOST_CHECK(meth.hasOutputVariable("prefix.Y"));

        double data[5];

        meth.extractResult("Y", (boost::uint8_t*)data, 5, 8, pdal::dimension::Float, 8);
        BOOST_CHECK(data[0] == 2.0);
        BOOST_CHECK(data[1] == 4.0);
        BOOST_CHECK(data[2] == 6.0);
        BOOST_CHECK(data[3] == 8.0);
        BOOST_CHECK(data[4] == 10.0);

        meth.extractResult("prefix.Y", (boost::uint8_t*)data, 5, 8, pdal::dimension::Float, 8);
        BOOST_CHECK(data[0] == 2.0);
        BOOST_CHECK(data[1] == 4.0);
        BOOST_CHECK(data[2] == 6.0);
        BOOST_CHECK(data[3] == 8.0);
        BOOST_CHECK(data[4] == 10.0);
    }

    {
        std::vector<std::string> names;
        meth.getOutputNames(names);
        BOOST_CHECK(names.size() == 2);
        BOOST_CHECK(names[0] == "Y");
        BOOST_CHECK(names[1] == "prefix.Y");
    }

    return;
}


BOOST_AUTO_TEST_CASE(PLangTest_returntrue)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  return True\n"
        ;
    pdal::plang::Script script(source, "MyTest", "yow");

    pdal::plang::Invocation meth(script);
    meth.compile();
    
    bool sts = meth.execute();
    BOOST_CHECK(sts);

    return;
}


BOOST_AUTO_TEST_CASE(PLangTest_returnfalse)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  return False\n"
        ;
    pdal::plang::Script script(source, "MyTest", "yow");

    pdal::plang::Invocation meth(script);
    meth.compile();
    
    bool sts = meth.execute();
    BOOST_CHECK(!sts);

    return;
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
    pdal::plang::Script script(source, "MyTest", "yow");

    double indata1[5] = {0.0, 1.0, 2.0, 3.0, 4.0};
    double outdata1[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double indata2[5] = {10.0, 20.0, 30.0, 40.0, 50.0};
    double outdata2[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

    pdal::plang::Invocation meth(script);
    meth.compile();
    
    {
        meth.insertArgument("X", (boost::uint8_t*)indata1, 5, 8, pdal::dimension::Float, 8);

        meth.execute();

        meth.extractResult("Y", (boost::uint8_t*)outdata1, 5, 8, pdal::dimension::Float, 8);

        BOOST_CHECK(outdata1[0] == 1.0);
        BOOST_CHECK(outdata1[1] == 2.0);
        BOOST_CHECK(outdata1[2] == 3.0);
        BOOST_CHECK(outdata1[3] == 4.0);
        BOOST_CHECK(outdata1[4] == 5.0);
    }

    {
        meth.insertArgument("X", (boost::uint8_t*)indata2, 5, 8, pdal::dimension::Float, 8);

        meth.execute();

        meth.extractResult("Y", (boost::uint8_t*)outdata2, 5, 8, pdal::dimension::Float, 8);

        BOOST_CHECK(outdata2[0] == 11.0);
        BOOST_CHECK(outdata2[1] == 21.0);
        BOOST_CHECK(outdata2[2] == 31.0);
        BOOST_CHECK(outdata2[3] == 41.0);
        BOOST_CHECK(outdata2[4] == 51.0);
    }

    return;
}


BOOST_AUTO_TEST_SUITE_END()

#endif
