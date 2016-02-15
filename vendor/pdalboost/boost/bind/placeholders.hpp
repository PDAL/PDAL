#ifndef BOOST_BIND_PLACEHOLDERS_HPP_INCLUDED
#define BOOST_BIND_PLACEHOLDERS_HPP_INCLUDED

// MS compatible compilers support #pragma once

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
# pragma once
#endif

//
//  bind/placeholders.hpp - _N definitions
//
//  Copyright (c) 2002 Peter Dimov and Multi Media Ltd.
//  Copyright 2015 Peter Dimov
//
//  Distributed under the Boost Software License, Version 1.0.
//  See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt
//
//  See http://www.boost.org/libs/bind/bind.html for documentation.
//

#include <boost/bind/arg.hpp>
#include <boost/config.hpp>

namespace pdalboost
{

namespace placeholders
{

#if defined(__BORLANDC__) || defined(__GNUC__) && (__GNUC__ < 4)

inline pdalboost::arg<1> _1() { return pdalboost::arg<1>(); }
inline pdalboost::arg<2> _2() { return pdalboost::arg<2>(); }
inline pdalboost::arg<3> _3() { return pdalboost::arg<3>(); }
inline pdalboost::arg<4> _4() { return pdalboost::arg<4>(); }
inline pdalboost::arg<5> _5() { return pdalboost::arg<5>(); }
inline pdalboost::arg<6> _6() { return pdalboost::arg<6>(); }
inline pdalboost::arg<7> _7() { return pdalboost::arg<7>(); }
inline pdalboost::arg<8> _8() { return pdalboost::arg<8>(); }
inline pdalboost::arg<9> _9() { return pdalboost::arg<9>(); }

#else

BOOST_STATIC_CONSTEXPR pdalboost::arg<1> _1;
BOOST_STATIC_CONSTEXPR pdalboost::arg<2> _2;
BOOST_STATIC_CONSTEXPR pdalboost::arg<3> _3;
BOOST_STATIC_CONSTEXPR pdalboost::arg<4> _4;
BOOST_STATIC_CONSTEXPR pdalboost::arg<5> _5;
BOOST_STATIC_CONSTEXPR pdalboost::arg<6> _6;
BOOST_STATIC_CONSTEXPR pdalboost::arg<7> _7;
BOOST_STATIC_CONSTEXPR pdalboost::arg<8> _8;
BOOST_STATIC_CONSTEXPR pdalboost::arg<9> _9;

#endif

} // namespace placeholders

} // namespace pdalboost

#endif // #ifndef BOOST_BIND_PLACEHOLDERS_HPP_INCLUDED
