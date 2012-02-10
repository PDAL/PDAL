//  (C) Copyright 2009-2011 Frederic Bron.
//
//  Use, modification and distribution are subject to the Boost Software License,
//  Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt).
//
//  See http://www.boost.org/libs/type_traits for most recent version including documentation.

#ifndef BOOST_TT_HAS_LEFT_SHIFT_HPP_INCLUDED
#define BOOST_TT_HAS_LEFT_SHIFT_HPP_INCLUDED

#define BOOST_TT_TRAIT_NAME has_left_shift
#define BOOST_TT_TRAIT_OP <<
#define BOOST_TT_FORBIDDEN_IF\
   ::pdalboost::type_traits::ice_or<\
      /* Lhs==fundamental and Rhs==fundamental and (Lhs!=integral or Rhs!=integral) */\
      ::pdalboost::type_traits::ice_and<\
         ::pdalboost::is_fundamental< Lhs_nocv >::value,\
         ::pdalboost::is_fundamental< Rhs_nocv >::value,\
         ::pdalboost::type_traits::ice_or<\
            ::pdalboost::type_traits::ice_not< ::pdalboost::is_integral< Lhs_noref >::value >::value,\
            ::pdalboost::type_traits::ice_not< ::pdalboost::is_integral< Rhs_noref >::value >::value\
         >::value\
      >::value,\
      /* Lhs==fundamental and Rhs==pointer */\
      ::pdalboost::type_traits::ice_and<\
         ::pdalboost::is_fundamental< Lhs_nocv >::value,\
         ::pdalboost::is_pointer< Rhs_noref >::value\
      >::value,\
      /* Rhs==fundamental and Lhs==pointer */\
      ::pdalboost::type_traits::ice_and<\
         ::pdalboost::is_fundamental< Rhs_nocv >::value,\
         ::pdalboost::is_pointer< Lhs_noref >::value\
      >::value,\
      /* Lhs==pointer and Rhs==pointer */\
      ::pdalboost::type_traits::ice_and<\
         ::pdalboost::is_pointer< Lhs_noref >::value,\
         ::pdalboost::is_pointer< Rhs_noref >::value\
      >::value\
   >::value


#include <boost/type_traits/detail/has_binary_operator.hpp>

#undef BOOST_TT_TRAIT_NAME
#undef BOOST_TT_TRAIT_OP
#undef BOOST_TT_FORBIDDEN_IF

#endif
