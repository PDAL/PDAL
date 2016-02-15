#ifndef BOOST_TYPE_TRAITS_COPY_CV_HPP_INCLUDED
#define BOOST_TYPE_TRAITS_COPY_CV_HPP_INCLUDED

//
//  Copyright 2015 Peter Dimov
//
//  Distributed under the Boost Software License, Version 1.0.
//  See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt
//

#include <boost/type_traits/is_const.hpp>
#include <boost/type_traits/is_volatile.hpp>
#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/add_volatile.hpp>
#include <boost/type_traits/conditional.hpp>

namespace pdalboost
{

template<class T, class U> struct copy_cv
{
private:

    typedef typename pdalboost::conditional<pdalboost::is_const<U>::value, typename pdalboost::add_const<T>::type, T>::type CT;

public:

    typedef typename pdalboost::conditional<pdalboost::is_volatile<U>::value, typename pdalboost::add_volatile<CT>::type, CT>::type type;
};

} // namespace pdalboost

#endif // #ifndef BOOST_TYPE_TRAITS_COPY_CV_HPP_INCLUDED
