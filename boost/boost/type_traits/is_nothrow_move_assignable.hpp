
//  (C) Copyright Steve Cleary, Beman Dawes, Howard Hinnant & John Maddock 2000.
//  (C) Copyright Eric Friedman 2002-2003.
//  (C) Copyright Antony Polukhin 2013.
//  Use, modification and distribution are subject to the Boost Software License,
//  Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt).
//
//  See http://www.boost.org/libs/type_traits for most recent version including documentation.

#ifndef BOOST_TT_IS_NOTHROW_MOVE_ASSIGNABLE_HPP_INCLUDED
#define BOOST_TT_IS_NOTHROW_MOVE_ASSIGNABLE_HPP_INCLUDED

#include <boost/config.hpp>
#include <boost/type_traits/has_trivial_move_assign.hpp>
#include <boost/type_traits/has_nothrow_assign.hpp>
#include <boost/type_traits/is_array.hpp>
#include <boost/type_traits/is_reference.hpp>
#include <boost/type_traits/detail/ice_and.hpp>
#include <boost/type_traits/detail/ice_or.hpp>
#include <boost/type_traits/detail/ice_not.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/utility/declval.hpp>

// should be the last #include
#include <boost/type_traits/detail/bool_trait_def.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {

namespace detail{

#ifndef BOOST_NO_CXX11_NOEXCEPT

template <class T, class Enable = void>
struct false_or_cpp11_noexcept_move_assignable: public ::pdalboost::false_type {};

template <class T>
struct false_or_cpp11_noexcept_move_assignable <
        T,
        typename ::pdalboost::enable_if_c<sizeof(T) && BOOST_NOEXCEPT_EXPR(::pdalboost::declval<T&>() = ::pdalboost::declval<T>())>::type
    > : public ::pdalboost::integral_constant<bool, BOOST_NOEXCEPT_EXPR(::pdalboost::declval<T&>() = ::pdalboost::declval<T>())>
{};

template <class T>
struct is_nothrow_move_assignable_imp{
    BOOST_STATIC_CONSTANT(bool, value = (
        ::pdalboost::type_traits::ice_and<
            ::pdalboost::type_traits::ice_not< ::pdalboost::is_volatile<T>::value >::value,
            ::pdalboost::type_traits::ice_not< ::pdalboost::is_reference<T>::value >::value,
            ::pdalboost::detail::false_or_cpp11_noexcept_move_assignable<T>::value
        >::value));
};

#else

template <class T>
struct is_nothrow_move_assignable_imp{
    BOOST_STATIC_CONSTANT(bool, value = (
        ::pdalboost::type_traits::ice_and<
            ::pdalboost::type_traits::ice_or<
                ::pdalboost::has_trivial_move_assign<T>::value,
                ::pdalboost::has_nothrow_assign<T>::value
            >::value,
            ::pdalboost::type_traits::ice_not< ::pdalboost::is_array<T>::value >::value
        >::value));
};

#endif

}

BOOST_TT_AUX_BOOL_TRAIT_DEF1(is_nothrow_move_assignable,T,::pdalboost::detail::is_nothrow_move_assignable_imp<T>::value)
BOOST_TT_AUX_BOOL_TRAIT_SPEC1(is_nothrow_move_assignable,void,false)
#ifndef BOOST_NO_CV_VOID_SPECIALIZATIONS
BOOST_TT_AUX_BOOL_TRAIT_SPEC1(is_nothrow_move_assignable,void const,false)
BOOST_TT_AUX_BOOL_TRAIT_SPEC1(is_nothrow_move_assignable,void const volatile,false)
BOOST_TT_AUX_BOOL_TRAIT_SPEC1(is_nothrow_move_assignable,void volatile,false)
#endif

} // namespace pdalboost

#include <boost/type_traits/detail/bool_trait_undef.hpp>

#endif // BOOST_TT_IS_NOTHROW_MOVE_ASSIGNABLE_HPP_INCLUDED
