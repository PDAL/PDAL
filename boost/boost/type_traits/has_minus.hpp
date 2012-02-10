//  (C) Copyright 2009-2011 Frederic Bron.
//
//  Use, modification and distribution are subject to the Boost Software License,
//  Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt).
//
//  See http://www.boost.org/libs/type_traits for most recent version including documentation.

#ifndef BOOST_TT_HAS_MINUS_HPP_INCLUDED
#define BOOST_TT_HAS_MINUS_HPP_INCLUDED

#define BOOST_TT_TRAIT_NAME has_minus
#define BOOST_TT_TRAIT_OP -
#define BOOST_TT_FORBIDDEN_IF\
   ::pdalboost::type_traits::ice_or<\
      /* Lhs==pointer and Rhs==fundamental and Rhs!=integral */\
      ::pdalboost::type_traits::ice_and<\
         ::pdalboost::is_pointer< Lhs_noref >::value,\
         ::pdalboost::is_fundamental< Rhs_nocv >::value,\
         ::pdalboost::type_traits::ice_not< ::pdalboost::is_integral< Rhs_noref >::value >::value\
      >::value,\
      /* Lhs==void* and (Rhs==fundamental or Rhs==pointer) */\
      ::pdalboost::type_traits::ice_and<\
         ::pdalboost::is_pointer< Lhs_noref >::value,\
         ::pdalboost::is_void< Lhs_noptr >::value,\
         ::pdalboost::type_traits::ice_or<\
            ::pdalboost::is_fundamental< Rhs_nocv >::value,\
            ::pdalboost::is_pointer< Rhs_noref >::value\
         >::value\
      >::value,\
      /* Rhs==void* and (Lhs==fundamental or Lhs==pointer) */\
      ::pdalboost::type_traits::ice_and<\
         ::pdalboost::is_pointer< Rhs_noref >::value,\
         ::pdalboost::is_void< Rhs_noptr >::value,\
         ::pdalboost::type_traits::ice_or<\
            ::pdalboost::is_fundamental< Lhs_nocv >::value,\
            ::pdalboost::is_pointer< Lhs_noref >::value\
         >::value\
      >::value,\
      /* Lhs=fundamental and Rhs=pointer */\
      ::pdalboost::type_traits::ice_and<\
         ::pdalboost::is_fundamental< Lhs_nocv >::value,\
         ::pdalboost::is_pointer< Rhs_noref >::value\
      >::value,\
      /* two different pointers */\
      ::pdalboost::type_traits::ice_and<\
         ::pdalboost::is_pointer< Lhs_noref >::value,\
         ::pdalboost::is_pointer< Rhs_noref >::value,\
         ::pdalboost::type_traits::ice_not< ::pdalboost::is_same< Lhs_nocv, Rhs_nocv >::value >::value\
      >::value\
   >::value


#include <boost/type_traits/detail/has_binary_operator.hpp>

#undef BOOST_TT_TRAIT_NAME
#undef BOOST_TT_TRAIT_OP
#undef BOOST_TT_FORBIDDEN_IF

#endif
