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
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
//  See http://www.boost.org/libs/bind/bind.html for documentation.
//

#include <boost/bind/arg.hpp>
#include <boost/config.hpp>

namespace
{

#if defined(__BORLANDC__) || defined(__GNUC__) && (__GNUC__ < 4)

static inline pdalboost::arg<1> _1() { return pdalboost::arg<1>(); }
static inline pdalboost::arg<2> _2() { return pdalboost::arg<2>(); }
static inline pdalboost::arg<3> _3() { return pdalboost::arg<3>(); }
static inline pdalboost::arg<4> _4() { return pdalboost::arg<4>(); }
static inline pdalboost::arg<5> _5() { return pdalboost::arg<5>(); }
static inline pdalboost::arg<6> _6() { return pdalboost::arg<6>(); }
static inline pdalboost::arg<7> _7() { return pdalboost::arg<7>(); }
static inline pdalboost::arg<8> _8() { return pdalboost::arg<8>(); }
static inline pdalboost::arg<9> _9() { return pdalboost::arg<9>(); }

#elif defined(BOOST_MSVC) || (defined(__DECCXX_VER) && __DECCXX_VER <= 60590031) || defined(__MWERKS__) || \
    defined(__GNUC__) && (__GNUC__ == 4 && __GNUC_MINOR__ < 2)  

static pdalboost::arg<1> _1;
static pdalboost::arg<2> _2;
static pdalboost::arg<3> _3;
static pdalboost::arg<4> _4;
static pdalboost::arg<5> _5;
static pdalboost::arg<6> _6;
static pdalboost::arg<7> _7;
static pdalboost::arg<8> _8;
static pdalboost::arg<9> _9;

#else

pdalboost::arg<1> _1;
pdalboost::arg<2> _2;
pdalboost::arg<3> _3;
pdalboost::arg<4> _4;
pdalboost::arg<5> _5;
pdalboost::arg<6> _6;
pdalboost::arg<7> _7;
pdalboost::arg<8> _8;
pdalboost::arg<9> _9;

#endif

} // unnamed namespace

#endif // #ifndef BOOST_BIND_PLACEHOLDERS_HPP_INCLUDED
