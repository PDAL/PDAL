// Copyright (C) 2012 Vicente J. Botet Escriba
//
//  Distributed under the Boost Software License, Version 1.0. (See accompanying
//  file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

// 2013/04 Vicente J. Botet Escriba
//    Provide implementation up to 9 parameters when BOOST_NO_CXX11_VARIADIC_TEMPLATES is defined.
//    Make use of Boost.Move
//    Make use of Boost.Tuple (movable)
// 2012/11 Vicente J. Botet Escriba
//    Adapt to boost libc++ implementation

//===----------------------------------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
// The async_func code is based on the one from libcxx.
//===----------------------------------------------------------------------===//

#ifndef BOOST_THREAD_DETAIL_ASYNC_FUNCT_HPP
#define BOOST_THREAD_DETAIL_ASYNC_FUNCT_HPP

#include <boost/config.hpp>

#include <boost/utility/result_of.hpp>
#include <boost/thread/detail/move.hpp>
#include <boost/thread/detail/invoke.hpp>
#include <boost/thread/detail/make_tuple_indices.hpp>

#if ! defined(BOOST_NO_CXX11_VARIADIC_TEMPLATES) && \
    ! defined(BOOST_NO_CXX11_HDR_TUPLE)
#include <tuple>
#else
#include <boost/tuple/tuple.hpp>
#endif

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost
{
  namespace detail
  {

#if ! defined(BOOST_NO_CXX11_VARIADIC_TEMPLATES) && \
    ! defined(BOOST_NO_CXX11_HDR_TUPLE)

    template <class Fp, class ... Args>
    class async_func
    {
      std::tuple<Fp, Args...> f_;

    public:
      BOOST_THREAD_MOVABLE_ONLY( async_func)
      //typedef typename invoke_of<_Fp, _Args...>::type Rp;
      typedef typename result_of<Fp(Args...)>::type result_type;

      BOOST_SYMBOL_VISIBLE
      explicit async_func(BOOST_THREAD_RV_REF(Fp) f, BOOST_THREAD_RV_REF(Args)... args)
      : f_(pdalboost::move(f), pdalboost::move(args)...)
      {}

      BOOST_SYMBOL_VISIBLE
      async_func(BOOST_THREAD_RV_REF(async_func) f) : f_(pdalboost::move(f.f_))
      {}

      result_type operator()()
      {
        typedef typename make_tuple_indices<1+sizeof...(Args), 1>::type Index;
        return execute(Index());
      }
    private:
      template <size_t ...Indices>
      result_type
      execute(tuple_indices<Indices...>)
      {
        return invoke(pdalboost::move(std::get<0>(f_)), pdalboost::move(std::get<Indices>(f_))...);
      }
    };
    //BOOST_THREAD_DCL_MOVABLE_BEG(X) async_func<Fp> BOOST_THREAD_DCL_MOVABLE_END
#else
    template <class Fp,
    class T0 = tuples::null_type, class T1 = tuples::null_type, class T2 = tuples::null_type,
    class T3 = tuples::null_type, class T4 = tuples::null_type, class T5 = tuples::null_type,
    class T6 = tuples::null_type, class T7 = tuples::null_type, class T8 = tuples::null_type
    , class T9 = tuples::null_type
    >
    class async_func;

    template <class Fp,
    class T0 , class T1 , class T2 ,
    class T3 , class T4 , class T5 ,
    class T6 , class T7 , class T8 >
    class async_func<Fp, T0, T1, T2, T3, T4, T5, T6, T7, T8>
    {
      Fp fp_;
      T0 v0_;
      T1 v1_;
      T2 v2_;
      T3 v3_;
      T4 v4_;
      T5 v5_;
      T6 v6_;
      T7 v7_;
      T8 v8_;
      //::pdalboost::tuple<Fp, T0, T1, T2, T3, T4, T5, T6, T7, T8> f_;

    public:
      BOOST_THREAD_MOVABLE_ONLY(async_func)
      typedef typename result_of<Fp(T0, T1, T2, T3, T4, T5, T6, T7, T8)>::type result_type;

      BOOST_SYMBOL_VISIBLE
      explicit async_func(BOOST_THREAD_RV_REF(Fp) f
          , BOOST_THREAD_RV_REF(T0) a0
          , BOOST_THREAD_RV_REF(T1) a1
          , BOOST_THREAD_RV_REF(T2) a2
          , BOOST_THREAD_RV_REF(T3) a3
          , BOOST_THREAD_RV_REF(T4) a4
          , BOOST_THREAD_RV_REF(T5) a5
          , BOOST_THREAD_RV_REF(T6) a6
          , BOOST_THREAD_RV_REF(T7) a7
          , BOOST_THREAD_RV_REF(T8) a8
      )
      : fp_(pdalboost::move(f))
      , v0_(pdalboost::move(a0))
      , v1_(pdalboost::move(a1))
      , v2_(pdalboost::move(a2))
      , v3_(pdalboost::move(a3))
      , v4_(pdalboost::move(a4))
      , v5_(pdalboost::move(a5))
      , v6_(pdalboost::move(a6))
      , v7_(pdalboost::move(a7))
      , v8_(pdalboost::move(a8))
      {}

      BOOST_SYMBOL_VISIBLE
      async_func(BOOST_THREAD_RV_REF(async_func) f)
      : fp_(pdalboost::move(f.fp))
      , v0_(pdalboost::move(f.a0))
      , v1_(pdalboost::move(f.a1))
      , v2_(pdalboost::move(f.a2))
      , v3_(pdalboost::move(f.a3))
      , v4_(pdalboost::move(f.a4))
      , v5_(pdalboost::move(f.a5))
      , v6_(pdalboost::move(f.a6))
      , v7_(pdalboost::move(f.a7))
      , v8_(pdalboost::move(f.a8))
      {}

      result_type operator()()
      {
        return invoke(pdalboost::move(fp_)
            , pdalboost::move(v0_)
            , pdalboost::move(v1_)
            , pdalboost::move(v2_)
            , pdalboost::move(v3_)
            , pdalboost::move(v4_)
            , pdalboost::move(v5_)
            , pdalboost::move(v6_)
            , pdalboost::move(v7_)
            , pdalboost::move(v8_)
        );
      }
    };
    template <class Fp, class T0, class T1, class T2, class T3, class T4, class T5, class T6, class T7 >
    class async_func<Fp, T0, T1, T2, T3, T4, T5, T6, T7>
    {
      Fp fp_;
      T0 v0_;
      T1 v1_;
      T2 v2_;
      T3 v3_;
      T4 v4_;
      T5 v5_;
      T6 v6_;
      T7 v7_;
    public:
      BOOST_THREAD_MOVABLE_ONLY(async_func)
      typedef typename result_of<Fp(T0, T1, T2, T3, T4, T5, T6, T7)>::type result_type;

      BOOST_SYMBOL_VISIBLE
      explicit async_func(BOOST_THREAD_RV_REF(Fp) f
          , BOOST_THREAD_RV_REF(T0) a0
          , BOOST_THREAD_RV_REF(T1) a1
          , BOOST_THREAD_RV_REF(T2) a2
          , BOOST_THREAD_RV_REF(T3) a3
          , BOOST_THREAD_RV_REF(T4) a4
          , BOOST_THREAD_RV_REF(T5) a5
          , BOOST_THREAD_RV_REF(T6) a6
          , BOOST_THREAD_RV_REF(T7) a7
      )
      : fp_(pdalboost::move(f))
      , v0_(pdalboost::move(a0))
      , v1_(pdalboost::move(a1))
      , v2_(pdalboost::move(a2))
      , v3_(pdalboost::move(a3))
      , v4_(pdalboost::move(a4))
      , v5_(pdalboost::move(a5))
      , v6_(pdalboost::move(a6))
      , v7_(pdalboost::move(a7))
      {}

      BOOST_SYMBOL_VISIBLE
      async_func(BOOST_THREAD_RV_REF(async_func) f)
      : fp_(pdalboost::move(f.fp))
      , v0_(pdalboost::move(f.a0))
      , v1_(pdalboost::move(f.a1))
      , v2_(pdalboost::move(f.a2))
      , v3_(pdalboost::move(f.a3))
      , v4_(pdalboost::move(f.a4))
      , v5_(pdalboost::move(f.a5))
      , v6_(pdalboost::move(f.a6))
      , v7_(pdalboost::move(f.a7))
      {}

      result_type operator()()
      {
        return invoke(pdalboost::move(fp_)
            , pdalboost::move(v0_)
            , pdalboost::move(v1_)
            , pdalboost::move(v2_)
            , pdalboost::move(v3_)
            , pdalboost::move(v4_)
            , pdalboost::move(v5_)
            , pdalboost::move(v6_)
            , pdalboost::move(v7_)
        );
      }
    };
    template <class Fp, class T0, class T1, class T2, class T3, class T4, class T5, class T6>
    class async_func<Fp, T0, T1, T2, T3, T4, T5, T6>
    {
      Fp fp_;
      T0 v0_;
      T1 v1_;
      T2 v2_;
      T3 v3_;
      T4 v4_;
      T5 v5_;
      T6 v6_;
    public:
      BOOST_THREAD_MOVABLE_ONLY(async_func)
      typedef typename result_of<Fp(T0, T1, T2, T3, T4, T5, T6)>::type result_type;

      BOOST_SYMBOL_VISIBLE
      explicit async_func(BOOST_THREAD_RV_REF(Fp) f
          , BOOST_THREAD_RV_REF(T0) a0
          , BOOST_THREAD_RV_REF(T1) a1
          , BOOST_THREAD_RV_REF(T2) a2
          , BOOST_THREAD_RV_REF(T3) a3
          , BOOST_THREAD_RV_REF(T4) a4
          , BOOST_THREAD_RV_REF(T5) a5
          , BOOST_THREAD_RV_REF(T6) a6
      )
      : fp_(pdalboost::move(f))
      , v0_(pdalboost::move(a0))
      , v1_(pdalboost::move(a1))
      , v2_(pdalboost::move(a2))
      , v3_(pdalboost::move(a3))
      , v4_(pdalboost::move(a4))
      , v5_(pdalboost::move(a5))
      , v6_(pdalboost::move(a6))
      {}

      BOOST_SYMBOL_VISIBLE
      async_func(BOOST_THREAD_RV_REF(async_func) f)
      : fp_(pdalboost::move(f.fp))
      , v0_(pdalboost::move(f.a0))
      , v1_(pdalboost::move(f.a1))
      , v2_(pdalboost::move(f.a2))
      , v3_(pdalboost::move(f.a3))
      , v4_(pdalboost::move(f.a4))
      , v5_(pdalboost::move(f.a5))
      , v6_(pdalboost::move(f.a6))
      {}

      result_type operator()()
      {
        return invoke(pdalboost::move(fp_)
            , pdalboost::move(v0_)
            , pdalboost::move(v1_)
            , pdalboost::move(v2_)
            , pdalboost::move(v3_)
            , pdalboost::move(v4_)
            , pdalboost::move(v5_)
            , pdalboost::move(v6_)
        );
      }
    };
    template <class Fp, class T0, class T1, class T2, class T3, class T4, class T5>
    class async_func<Fp, T0, T1, T2, T3, T4, T5>
    {
      Fp fp_;
      T0 v0_;
      T1 v1_;
      T2 v2_;
      T3 v3_;
      T4 v4_;
      T5 v5_;
    public:
      BOOST_THREAD_MOVABLE_ONLY(async_func)
      typedef typename result_of<Fp(T0, T1, T2, T3, T4, T5)>::type result_type;

      BOOST_SYMBOL_VISIBLE
      explicit async_func(BOOST_THREAD_RV_REF(Fp) f
          , BOOST_THREAD_RV_REF(T0) a0
          , BOOST_THREAD_RV_REF(T1) a1
          , BOOST_THREAD_RV_REF(T2) a2
          , BOOST_THREAD_RV_REF(T3) a3
          , BOOST_THREAD_RV_REF(T4) a4
          , BOOST_THREAD_RV_REF(T5) a5
      )
      : fp_(pdalboost::move(f))
      , v0_(pdalboost::move(a0))
      , v1_(pdalboost::move(a1))
      , v2_(pdalboost::move(a2))
      , v3_(pdalboost::move(a3))
      , v4_(pdalboost::move(a4))
      , v5_(pdalboost::move(a5))
      {}

      BOOST_SYMBOL_VISIBLE
      async_func(BOOST_THREAD_RV_REF(async_func) f)
      : fp_(pdalboost::move(f.fp))
      , v0_(pdalboost::move(f.a0))
      , v1_(pdalboost::move(f.a1))
      , v2_(pdalboost::move(f.a2))
      , v3_(pdalboost::move(f.a3))
      , v4_(pdalboost::move(f.a4))
      , v5_(pdalboost::move(f.a5))
      {}

      result_type operator()()
      {
        return invoke(pdalboost::move(fp_)
            , pdalboost::move(v0_)
            , pdalboost::move(v1_)
            , pdalboost::move(v2_)
            , pdalboost::move(v3_)
            , pdalboost::move(v4_)
            , pdalboost::move(v5_)
        );
      }
    };
    template <class Fp, class T0, class T1, class T2, class T3, class T4>
    class async_func<Fp, T0, T1, T2, T3, T4>
    {
      Fp fp_;
      T0 v0_;
      T1 v1_;
      T2 v2_;
      T3 v3_;
      T4 v4_;
    public:
      BOOST_THREAD_MOVABLE_ONLY(async_func)
      typedef typename result_of<Fp(T0, T1, T2, T3, T4)>::type result_type;

      BOOST_SYMBOL_VISIBLE
      explicit async_func(BOOST_THREAD_RV_REF(Fp) f
          , BOOST_THREAD_RV_REF(T0) a0
          , BOOST_THREAD_RV_REF(T1) a1
          , BOOST_THREAD_RV_REF(T2) a2
          , BOOST_THREAD_RV_REF(T3) a3
          , BOOST_THREAD_RV_REF(T4) a4
      )
      : fp_(pdalboost::move(f))
      , v0_(pdalboost::move(a0))
      , v1_(pdalboost::move(a1))
      , v2_(pdalboost::move(a2))
      , v3_(pdalboost::move(a3))
      , v4_(pdalboost::move(a4))
      {}

      BOOST_SYMBOL_VISIBLE
      async_func(BOOST_THREAD_RV_REF(async_func) f)
      : fp_(pdalboost::move(f.fp))
      , v0_(pdalboost::move(f.a0))
      , v1_(pdalboost::move(f.a1))
      , v2_(pdalboost::move(f.a2))
      , v3_(pdalboost::move(f.a3))
      , v4_(pdalboost::move(f.a4))
      {}

      result_type operator()()
      {
        return invoke(pdalboost::move(fp_)
            , pdalboost::move(v0_)
            , pdalboost::move(v1_)
            , pdalboost::move(v2_)
            , pdalboost::move(v3_)
            , pdalboost::move(v4_)
        );
      }
    };
    template <class Fp, class T0, class T1, class T2, class T3>
    class async_func<Fp, T0, T1, T2, T3>
    {
      Fp fp_;
      T0 v0_;
      T1 v1_;
      T2 v2_;
      T3 v3_;
    public:
      BOOST_THREAD_MOVABLE_ONLY(async_func)
      typedef typename result_of<Fp(T0, T1, T2, T3)>::type result_type;

      BOOST_SYMBOL_VISIBLE
      explicit async_func(BOOST_THREAD_RV_REF(Fp) f
          , BOOST_THREAD_RV_REF(T0) a0
          , BOOST_THREAD_RV_REF(T1) a1
          , BOOST_THREAD_RV_REF(T2) a2
          , BOOST_THREAD_RV_REF(T3) a3
      )
      : fp_(pdalboost::move(f))
      , v0_(pdalboost::move(a0))
      , v1_(pdalboost::move(a1))
      , v2_(pdalboost::move(a2))
      , v3_(pdalboost::move(a3))
      {}

      BOOST_SYMBOL_VISIBLE
      async_func(BOOST_THREAD_RV_REF(async_func) f)
      : fp_(pdalboost::move(f.fp))
      , v0_(pdalboost::move(f.a0))
      , v1_(pdalboost::move(f.a1))
      , v2_(pdalboost::move(f.a2))
      , v3_(pdalboost::move(f.a3))
      {}

      result_type operator()()
      {
        return invoke(pdalboost::move(fp_)
            , pdalboost::move(v0_)
            , pdalboost::move(v1_)
            , pdalboost::move(v2_)
            , pdalboost::move(v3_)
        );
      }
    };
    template <class Fp, class T0, class T1, class T2>
    class async_func<Fp, T0, T1, T2>
    {
      Fp fp_;
      T0 v0_;
      T1 v1_;
      T2 v2_;
    public:
      BOOST_THREAD_MOVABLE_ONLY(async_func)
      typedef typename result_of<Fp(T0, T1, T2)>::type result_type;

      BOOST_SYMBOL_VISIBLE
      explicit async_func(BOOST_THREAD_RV_REF(Fp) f
          , BOOST_THREAD_RV_REF(T0) a0
          , BOOST_THREAD_RV_REF(T1) a1
          , BOOST_THREAD_RV_REF(T2) a2
      )
      : fp_(pdalboost::move(f))
      , v0_(pdalboost::move(a0))
      , v1_(pdalboost::move(a1))
      , v2_(pdalboost::move(a2))
      {}

      BOOST_SYMBOL_VISIBLE
      async_func(BOOST_THREAD_RV_REF(async_func) f)
      : fp_(pdalboost::move(f.fp))
      , v0_(pdalboost::move(f.a0))
      , v1_(pdalboost::move(f.a1))
      , v2_(pdalboost::move(f.a2))
      {}

      result_type operator()()
      {
        return invoke(pdalboost::move(fp_)
            , pdalboost::move(v0_)
            , pdalboost::move(v1_)
            , pdalboost::move(v2_)
        );
      }
    };
    template <class Fp, class T0, class T1>
    class async_func<Fp, T0, T1>
    {
      Fp fp_;
      T0 v0_;
      T1 v1_;
    public:
      BOOST_THREAD_MOVABLE_ONLY(async_func)
      typedef typename result_of<Fp(T0, T1)>::type result_type;

      BOOST_SYMBOL_VISIBLE
      explicit async_func(BOOST_THREAD_RV_REF(Fp) f
          , BOOST_THREAD_RV_REF(T0) a0
          , BOOST_THREAD_RV_REF(T1) a1
      )
      : fp_(pdalboost::move(f))
      , v0_(pdalboost::move(a0))
      , v1_(pdalboost::move(a1))
      {}

      BOOST_SYMBOL_VISIBLE
      async_func(BOOST_THREAD_RV_REF(async_func) f)
      : fp_(pdalboost::move(f.fp))
      , v0_(pdalboost::move(f.a0))
      , v1_(pdalboost::move(f.a1))
      {}

      result_type operator()()
      {
        return invoke(pdalboost::move(fp_)
            , pdalboost::move(v0_)
            , pdalboost::move(v1_)
        );
      }
    };
    template <class Fp, class T0>
    class async_func<Fp, T0>
    {
      Fp fp_;
      T0 v0_;
    public:
      BOOST_THREAD_MOVABLE_ONLY(async_func)
      typedef typename result_of<Fp(T0)>::type result_type;

      BOOST_SYMBOL_VISIBLE
      explicit async_func(BOOST_THREAD_RV_REF(Fp) f
          , BOOST_THREAD_RV_REF(T0) a0
      )
      : fp_(pdalboost::move(f))
      , v0_(pdalboost::move(a0))
      {}

      BOOST_SYMBOL_VISIBLE
      async_func(BOOST_THREAD_RV_REF(async_func) f)
      : fp_(pdalboost::move(f.fp))
      , v0_(pdalboost::move(f.a0))
      {}

      result_type operator()()
      {
        return invoke(pdalboost::move(fp_)
            , pdalboost::move(v0_)
        );
      }
    };
    template <class Fp>
    class async_func<Fp>
    {
      Fp fp_;
    public:
      BOOST_THREAD_COPYABLE_AND_MOVABLE(async_func)
      typedef typename result_of<Fp()>::type result_type;
      BOOST_SYMBOL_VISIBLE
      explicit async_func(BOOST_THREAD_FWD_REF(Fp) f)
      : fp_(pdalboost::move(f))
      {}

      BOOST_SYMBOL_VISIBLE
      async_func(BOOST_THREAD_FWD_REF(async_func) f)
      : fp_(pdalboost::move(f.fp_))
      {}
      result_type operator()()
      {
        return fp_();
      }
    };
#endif

  }
}

#endif // header
