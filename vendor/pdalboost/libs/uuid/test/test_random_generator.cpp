//  (C) Copyright Andy Tompkins 2010. Permission to copy, use, modify, sell and
//  distribute this software is granted provided this copyright notice appears
//  in all copies. This software is provided "as is" without express or implied
//  warranty, and with no claim as to its suitability for any purpose.

// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

//  libs/uuid/test/test_random_generator.cpp  -------------------------------//

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/detail/lightweight_test.hpp>
#include <boost/random.hpp>

template <typename RandomUuidGenerator>
void check_random_generator(RandomUuidGenerator& uuid_gen)
{
    pdalboost::uuids::uuid u1 = uuid_gen();
    pdalboost::uuids::uuid u2 = uuid_gen();

    BOOST_TEST_NE(u1, u2);

    // check variant
    BOOST_TEST_EQ(u1.variant(), pdalboost::uuids::uuid::variant_rfc_4122);

    // version
    BOOST_TEST_EQ(u1.version(), pdalboost::uuids::uuid::version_random_number_based);
}

int main(int, char*[])
{
    using namespace pdalboost::uuids;

    // default random number generator
    random_generator uuid_gen1;
    check_random_generator(uuid_gen1);
    
    // specific random number generator
    basic_random_generator<pdalboost::rand48> uuid_gen2;
    check_random_generator(uuid_gen2);
    
    // pass by reference
    pdalboost::ecuyer1988 ecuyer1988_gen;
    basic_random_generator<pdalboost::ecuyer1988> uuid_gen3(ecuyer1988_gen);
    check_random_generator(uuid_gen3);

    // pass by pointer
    pdalboost::lagged_fibonacci607 lagged_fibonacci607_gen;
    basic_random_generator<pdalboost::lagged_fibonacci607> uuid_gen4(&lagged_fibonacci607_gen);
    check_random_generator(uuid_gen4);

    // random device
    //basic_random_generator<pdalboost::random_device> uuid_gen5;
    //check_random_generator(uuid_gen5);
    
    // there was a bug in basic_random_generator where it did not
    // produce very random numbers.  This checks for that bug.
    uuid u = random_generator()();
    if ( (u.data[4] == u.data[12]) &&
         (u.data[5] == u.data[9] && u.data[5] == u.data[13]) &&
         (u.data[7] == u.data[11] && u.data[7] == u.data[15]) &&
         (u.data[10] == u.data[14]) )
    {
        BOOST_ERROR("basic_random_generator is not producing random uuids");
    }

    return pdalboost::report_errors();
}
