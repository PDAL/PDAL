/*=============================================================================
    Copyright (c) 2001-2011 Joel de Guzman

    Distributed under the Boost Software License, Version 1.0. (See accompanying 
    file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
==============================================================================*/
#include <boost/detail/lightweight_test.hpp>
#include <boost/fusion/container/vector/vector.hpp>
#include <boost/fusion/adapted/mpl.hpp>
#include <boost/fusion/container/set/set.hpp>
#include <boost/fusion/container/map/map.hpp>
#include <boost/fusion/algorithm/query/find.hpp>
#include <boost/fusion/iterator/deref.hpp>
#include <boost/mpl/vector.hpp>
#include <string>

struct X
{
    operator int() const
    {
        return 12345;
    }
};
int
main()
{
    using namespace pdalboost::fusion;
    using pdalboost::mpl::identity;

    {
        typedef vector<int, char, int, double> seq_type;
        seq_type seq(12345, 'x', 678910, 3.36);

        std::cout << *pdalboost::fusion::find<char>(seq) << std::endl;
        BOOST_TEST(*pdalboost::fusion::find<char>(seq) == 'x');

        std::cout << *pdalboost::fusion::find<int>(seq) << std::endl;
        BOOST_TEST(*pdalboost::fusion::find<int>(seq) == 12345);

        std::cout << *pdalboost::fusion::find<double>(seq) << std::endl;
        BOOST_TEST(*pdalboost::fusion::find<double>(seq) == 3.36);

        BOOST_TEST(pdalboost::fusion::find<bool>(seq) == pdalboost::fusion::end(seq));
    }

    {
        typedef set<int, char, double> seq_type;
        seq_type seq(12345, 'x', 3.36);
        std::cout << *pdalboost::fusion::find<char>(seq) << std::endl;
        BOOST_TEST(*pdalboost::fusion::find<char>(seq) == 'x');
        BOOST_TEST(pdalboost::fusion::find<bool>(seq) == pdalboost::fusion::end(seq));
    }
    
    {
        typedef map<
            pair<int, char>
          , pair<double, std::string> > 
        map_type;
        
        map_type seq(
            make_pair<int>('X')
          , make_pair<double>("Men"));
        
        std::cout << *pdalboost::fusion::find<int>(seq) << std::endl;
        std::cout << *pdalboost::fusion::find<double>(seq) << std::endl;
        BOOST_TEST((*pdalboost::fusion::find<int>(seq)).second == 'X');
        BOOST_TEST((*pdalboost::fusion::find<double>(seq)).second == "Men");
        BOOST_TEST(pdalboost::fusion::find<bool>(seq) == pdalboost::fusion::end(seq));
    }

    {
        typedef pdalboost::mpl::vector<int, char, X, double> mpl_vec;
        BOOST_TEST((*pdalboost::fusion::find<X>(mpl_vec()) == 12345));
        BOOST_TEST(pdalboost::fusion::find<bool>(mpl_vec()) == pdalboost::fusion::end(mpl_vec()));
    }

    return pdalboost::report_errors();
}

