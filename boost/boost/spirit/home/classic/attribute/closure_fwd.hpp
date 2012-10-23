/*=============================================================================
    Copyright (c) 2006 Tobias Schwinger
    http://spirit.sourceforge.net/

  Distributed under the Boost Software License, Version 1.0. (See accompanying
  file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
=============================================================================*/
#if !defined(BOOST_SPIRIT_CLOSURE_FWD_HPP)
#define BOOST_SPIRIT_CLOSURE_FWD_HPP

#include <boost/spirit/home/classic/namespace.hpp>
#include <boost/spirit/home/classic/phoenix/tuples.hpp>

#if !defined(BOOST_SPIRIT_CLOSURE_LIMIT)
#   define BOOST_SPIRIT_CLOSURE_LIMIT PHOENIX_LIMIT
#endif

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost { namespace spirit {

BOOST_SPIRIT_CLASSIC_NAMESPACE_BEGIN

    template<typename ClosureT>
    class closure_context;

    template <typename ClosureT>
    class init_closure_context;

    template <typename ParserT, typename ActorTupleT>
    struct init_closure_parser;

    template <
            typename DerivedT
        ,   typename T0 = ::pdalboostphoenix::nil_t
        ,   typename T1 = ::pdalboostphoenix::nil_t
        ,   typename T2 = ::pdalboostphoenix::nil_t

    #if BOOST_SPIRIT_CLOSURE_LIMIT > 3
        ,   typename T3 = ::pdalboostphoenix::nil_t
        ,   typename T4 = ::pdalboostphoenix::nil_t
        ,   typename T5 = ::pdalboostphoenix::nil_t

    #if BOOST_SPIRIT_CLOSURE_LIMIT > 6
        ,   typename T6 = ::pdalboostphoenix::nil_t
        ,   typename T7 = ::pdalboostphoenix::nil_t
        ,   typename T8 = ::pdalboostphoenix::nil_t

    #if BOOST_SPIRIT_CLOSURE_LIMIT > 9
        ,   typename T9 = ::pdalboostphoenix::nil_t
        ,   typename T10 = ::pdalboostphoenix::nil_t
        ,   typename T11 = ::pdalboostphoenix::nil_t

    #if BOOST_SPIRIT_CLOSURE_LIMIT > 12
        ,   typename T12 = ::pdalboostphoenix::nil_t
        ,   typename T13 = ::pdalboostphoenix::nil_t
        ,   typename T14 = ::pdalboostphoenix::nil_t

    #endif
    #endif
    #endif
    #endif
    >
    struct closure;

BOOST_SPIRIT_CLASSIC_NAMESPACE_END

}} // namespace BOOST_SPIRIT_CLASSIC_NS

#endif

