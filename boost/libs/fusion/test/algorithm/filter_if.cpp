/*=============================================================================
    Copyright (c) 2001-2011 Joel de Guzman

    Distributed under the Boost Software License, Version 1.0. (See accompanying 
    file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
==============================================================================*/
#include <boost/detail/lightweight_test.hpp>
#include <boost/fusion/container/vector/vector.hpp>
#include <boost/fusion/adapted/mpl.hpp>
#include <boost/fusion/sequence/io/out.hpp>
#include <boost/fusion/container/generation/make_vector.hpp>
#include <boost/fusion/sequence/comparison/equal_to.hpp>
#include <boost/fusion/algorithm/transformation/filter_if.hpp>
#include <boost/type_traits/is_class.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/not.hpp>

struct X
{
    operator char const*() const
    {
        return "<X-object>";
    }
};

struct Y
{
    operator char const*() const
    {
        return "<Y-object>";
    }
};

int
main()
{
    using namespace pdalboost::fusion;

    using pdalboost::mpl::_;
    using pdalboost::mpl::not_;
    using pdalboost::is_class;
    using pdalboost::is_same;

    std::cout << tuple_open('[');
    std::cout << tuple_close(']');
    std::cout << tuple_delimiter(", ");

/// Testing filter_if

    X x; Y y;
    typedef pdalboost::fusion::vector<Y, char, long, X, bool, double> vector_type;
    vector_type t(y, '@', 987654, x, true, 6.6);

    {
        std::cout << filter_if<not_<is_class<_> > >(t) << std::endl;
        BOOST_TEST((filter_if<not_<is_class<_> > >(t)
            == make_vector('@', 987654, true, 6.6)));
    }

    {
        std::cout << filter_if<is_class<_> >(t) << std::endl;
        BOOST_TEST((filter_if<is_class<_> >(t)
            == make_vector(y, x)));
    }

    {
        typedef pdalboost::mpl::vector<Y, char, long, X, bool> mpl_vec;
        BOOST_TEST((filter_if<not_<is_class<_> > >(mpl_vec())
            == make_vector(char(), long(), bool())));
        BOOST_TEST((filter_if<is_class<_> >(mpl_vec())
            == make_vector(y, x)));
    }

    return pdalboost::report_errors();
}

