//  (C) Copyright John Maddock 2005.
//  Use, modification and distribution are subject to the
//  Boost Software License, Version 1.0. (See accompanying file
//  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_TR1_TUPLE_HPP_INCLUDED
#  define BOOST_TR1_TUPLE_HPP_INCLUDED
#  include <boost/tr1/detail/config.hpp>

#ifdef BOOST_HAS_TR1_TUPLE

#  if defined(BOOST_HAS_INCLUDE_NEXT) && !defined(BOOST_TR1_DISABLE_INCLUDE_NEXT)
#     include_next BOOST_TR1_HEADER(tuple)
#  else
#     include <boost/tr1/detail/config_all.hpp>
#     include BOOST_TR1_STD_HEADER(BOOST_TR1_PATH(tuple))
#  endif

#else

#if defined(BOOST_TR1_USE_OLD_TUPLE)

#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/type_traits/integral_constant.hpp>

namespace std{ namespace tr1{

using ::pdalboost::tuple;

// [6.1.3.2] Tuple creation functions
using ::pdalboost::tuples::ignore;
using ::pdalboost::make_tuple;
using ::pdalboost::tie;

// [6.1.3.3] Tuple helper classes
template <class T> 
struct tuple_size 
   : public ::pdalboost::integral_constant
   < ::std::size_t, ::pdalboost::tuples::length<T>::value>
{};

template < int I, class T>
struct tuple_element
{
   typedef typename pdalboost::tuples::element<I,T>::type type;
};

#if !BOOST_WORKAROUND(__BORLANDC__, < 0x0582)
// [6.1.3.4] Element access
using ::pdalboost::get;
#endif

} } // namespaces

#else

#include <boost/fusion/include/tuple.hpp>
#include <boost/fusion/include/std_pair.hpp>

namespace std{ namespace tr1{

using ::pdalboost::fusion::tuple;

// [6.1.3.2] Tuple creation functions
using ::pdalboost::fusion::ignore;
using ::pdalboost::fusion::make_tuple;
using ::pdalboost::fusion::tie;
using ::pdalboost::fusion::get;

// [6.1.3.3] Tuple helper classes
using ::pdalboost::fusion::tuple_size;
using ::pdalboost::fusion::tuple_element;

}}

#endif

#endif

#endif

