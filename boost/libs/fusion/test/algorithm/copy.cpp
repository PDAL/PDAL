/*=============================================================================
    Copyright (c) 2001-2011 Joel de Guzman

    Distributed under the Boost Software License, Version 1.0. (See accompanying
    file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
==============================================================================*/
#include <boost/detail/lightweight_test.hpp>
#include <boost/fusion/container/vector/vector.hpp>
#include <boost/fusion/container/list/list.hpp>
#include <boost/fusion/sequence/comparison.hpp>
#include <boost/fusion/algorithm/auxiliary/copy.hpp>

int
main()
{
    {
        pdalboost::fusion::vector<int, short, double> v(1, 2, 3);
        pdalboost::fusion::list<int, short, double> l;

        pdalboost::fusion::copy(v, l);
        BOOST_TEST(v == l);
    }

    return pdalboost::report_errors();
}

