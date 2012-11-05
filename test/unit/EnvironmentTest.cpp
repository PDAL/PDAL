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
 
#include <boost/test/unit_test.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/random/mersenne_twister.hpp>

#include <pdal/GlobalEnvironment.hpp>
#include "Support.hpp"


BOOST_AUTO_TEST_SUITE(EnvironmentTest)


BOOST_AUTO_TEST_CASE(EnvironmentTest_1)
{
    ::pdal::GlobalEnvironment& pdal_env = ::pdal::GlobalEnvironment::get();

#ifdef PDAL_HAVE_PYTHON
    ::pdal::plang::PythonEnvironment& python_env = pdal_env.getPythonEnvironment();

    (void)python_env;
#endif

    return;
}

boost::test_tools::predicate_result
compare_uuids( boost::uuids::uuid const& a, boost::uuids::uuid const& b )
{
    if (a.is_nil())
    {
        boost::test_tools::predicate_result res( false );

        res.message() << "UUID a was nil";

        return res;
    
    }

    if (b.is_nil())
    {
        boost::test_tools::predicate_result res( false );

        res.message() << "UUID b was nil";

        return res;
    
    }

    if (a == b)
    {
        boost::test_tools::predicate_result res( false );

        res.message() << "UUID A and B are equal with value '" << a << "'";

        return res;
    }

    return true;
}


BOOST_AUTO_TEST_CASE(EnvironmentTest_rng)
{
    ::pdal::GlobalEnvironment& env = ::pdal::GlobalEnvironment::get();

    boost::uuids::basic_random_generator<boost::mt19937> gen1(env.getRNG());
    boost::uuids::uuid a = gen1();
    
    boost::uuids::basic_random_generator<boost::mt19937> gen2(env.getRNG());
    boost::uuids::uuid b = gen2();    

    BOOST_CHECK( compare_uuids(a, b));
    
    boost::uuids::uuid c = gen1();    
    boost::uuids::uuid d = gen1();    

    BOOST_CHECK( compare_uuids(c, d));
    
    // This stuff should fail because both RNGs end up being initialized with 
    // the same seed (nothing)
    // boost::uuids::uuid e;
    // boost::uuids::uuid f;
    // {
    //     boost::mt19937 rng;
    //     boost::uuids::basic_random_generator<boost::mt19937> gen(&rng);
    //     e = gen();    
    // }
    // {
    //     boost::mt19937 rng;
    //     boost::uuids::basic_random_generator<boost::mt19937> gen(&rng);
    //     f = gen();    
    // }
    // 
    // BOOST_CHECK(compare_uuids(e, f));    
    
    return;
}


boost::test_tools::predicate_result
check_uuid_map( std::map<boost::uuids::uuid, int> ids)
{
    typedef std::map<boost::uuids::uuid, int>::const_iterator Iterator;
    
    for (Iterator i = ids.begin(); i != ids.end(); i++)
    {
        if (i->second > 1)
        {
            boost::test_tools::predicate_result res( false );

            res.message() << "UUID '" << i->first << "' has a count of " << i->second << " when it should be 1!";

            return res;            
        }
    }
    
    return true;
    
}

BOOST_AUTO_TEST_CASE(test_uuid_collision)
{
    std::map<boost::uuids::uuid, int> ids;
    typedef std::map<boost::uuids::uuid, int>::iterator Iterator;
    pdal::GlobalEnvironment& env = pdal::GlobalEnvironment::get();
    boost::uuids::basic_random_generator<boost::mt19937> gen(env.getRNG());
    
    boost::uint32_t test_size(10000);
    
    for (unsigned i = 0; i < test_size; ++i)
    {
        boost::uuids::uuid id = gen();
        Iterator it = ids.find(id);
        if (it != ids.end())
        {
            it->second++;
        } else
        {
            ids.insert(std::pair<boost::uuids::uuid, int>(id, 1));
        }
    }


}


BOOST_AUTO_TEST_SUITE_END()
