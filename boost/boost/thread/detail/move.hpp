// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
// (C) Copyright 2007-8 Anthony Williams
// (C) Copyright 2011-2012 Vicente J. Botet Escriba

#ifndef BOOST_THREAD_MOVE_HPP
#define BOOST_THREAD_MOVE_HPP

#include <boost/thread/detail/config.hpp>
#ifndef BOOST_NO_SFINAE
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_convertible.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <boost/type_traits/remove_cv.hpp>
#include <boost/type_traits/decay.hpp>
#endif

#include <boost/thread/detail/delete.hpp>
#include <boost/move/utility.hpp>
#include <boost/config/abi_prefix.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost
{

    namespace detail
    {
      template <typename T>
      struct enable_move_utility_emulation_dummy_specialization;
        template<typename T>
        struct thread_move_t
        {
            T& t;
            explicit thread_move_t(T& t_):
                t(t_)
            {}

            T& operator*() const
            {
                return t;
            }

            T* operator->() const
            {
                return &t;
            }
        private:
            void operator=(thread_move_t&);
        };
    }

#if !defined BOOST_THREAD_USES_MOVE

#ifndef BOOST_NO_SFINAE
    template<typename T>
    typename enable_if<pdalboost::is_convertible<T&,pdalboost::detail::thread_move_t<T> >, pdalboost::detail::thread_move_t<T> >::type move(T& t)
    {
        return pdalboost::detail::thread_move_t<T>(t);
    }
#endif

    template<typename T>
    pdalboost::detail::thread_move_t<T> move(pdalboost::detail::thread_move_t<T> t)
    {
        return t;
    }

#endif   //#if !defined BOOST_THREAD_USES_MOVE
}

#if ! defined  BOOST_NO_CXX11_RVALUE_REFERENCES

#define BOOST_THREAD_RV_REF(TYPE) BOOST_RV_REF(TYPE)
#define BOOST_THREAD_RV_REF_2_TEMPL_ARGS(TYPE) BOOST_RV_REF_2_TEMPL_ARGS(TYPE)
#define BOOST_THREAD_RV_REF_BEG BOOST_RV_REF_BEG
#define BOOST_THREAD_RV_REF_END BOOST_RV_REF_END
#define BOOST_THREAD_RV(V) V
#define BOOST_THREAD_MAKE_RV_REF(RVALUE) RVALUE
#define BOOST_THREAD_FWD_REF(TYPE) BOOST_FWD_REF(TYPE)
#define BOOST_THREAD_DCL_MOVABLE(TYPE)
#define BOOST_THREAD_DCL_MOVABLE_BEG(T) \
  namespace detail { \
    template <typename T> \
    struct enable_move_utility_emulation_dummy_specialization<

#define BOOST_THREAD_DCL_MOVABLE_END > \
      : integral_constant<bool, false> \
      {}; \
    }

#elif ! defined  BOOST_NO_CXX11_RVALUE_REFERENCES && defined  BOOST_MSVC

#define BOOST_THREAD_RV_REF(TYPE) BOOST_RV_REF(TYPE)
#define BOOST_THREAD_RV_REF_2_TEMPL_ARGS(TYPE) BOOST_RV_REF_2_TEMPL_ARGS(TYPE)
#define BOOST_THREAD_RV_REF_BEG BOOST_RV_REF_BEG
#define BOOST_THREAD_RV_REF_END BOOST_RV_REF_END
#define BOOST_THREAD_RV(V) V
#define BOOST_THREAD_MAKE_RV_REF(RVALUE) RVALUE
#define BOOST_THREAD_FWD_REF(TYPE) BOOST_FWD_REF(TYPE)
#define BOOST_THREAD_DCL_MOVABLE(TYPE)
#define BOOST_THREAD_DCL_MOVABLE_BEG(T) \
  namespace detail { \
    template <typename T> \
    struct enable_move_utility_emulation_dummy_specialization<

#define BOOST_THREAD_DCL_MOVABLE_END > \
      : integral_constant<bool, false> \
      {}; \
    }

#else

#if defined BOOST_THREAD_USES_MOVE
#define BOOST_THREAD_RV_REF(TYPE) BOOST_RV_REF(TYPE)
#define BOOST_THREAD_RV_REF_2_TEMPL_ARGS(TYPE) BOOST_RV_REF_2_TEMPL_ARGS(TYPE)
#define BOOST_THREAD_RV_REF_BEG BOOST_RV_REF_BEG
#define BOOST_THREAD_RV_REF_END BOOST_RV_REF_END
#define BOOST_THREAD_RV(V) V
#define BOOST_THREAD_FWD_REF(TYPE) BOOST_FWD_REF(TYPE)
#define BOOST_THREAD_DCL_MOVABLE(TYPE)
#define BOOST_THREAD_DCL_MOVABLE_BEG(T) \
  namespace detail { \
    template <typename T> \
    struct enable_move_utility_emulation_dummy_specialization<

#define BOOST_THREAD_DCL_MOVABLE_END > \
      : integral_constant<bool, false> \
      {}; \
    }

#else

#define BOOST_THREAD_RV_REF(TYPE) pdalboost::detail::thread_move_t< TYPE >
#define BOOST_THREAD_RV_REF_BEG pdalboost::detail::thread_move_t<
#define BOOST_THREAD_RV_REF_END >
#define BOOST_THREAD_RV(V) (*V)
#define BOOST_THREAD_FWD_REF(TYPE) BOOST_FWD_REF(TYPE)

#define BOOST_THREAD_DCL_MOVABLE(TYPE) \
template <> \
struct enable_move_utility_emulation< TYPE > \
{ \
   static const bool value = false; \
};

#define BOOST_THREAD_DCL_MOVABLE_BEG(T) \
template <typename T> \
struct enable_move_utility_emulation<

#define BOOST_THREAD_DCL_MOVABLE_END > \
{ \
   static const bool value = false; \
};

#endif

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost
{
namespace detail
{
  template <typename T>
  BOOST_THREAD_RV_REF(typename ::pdalboost::remove_cv<typename ::pdalboost::remove_reference<T>::type>::type)
  make_rv_ref(T v)  BOOST_NOEXCEPT
  {
    return (BOOST_THREAD_RV_REF(typename ::pdalboost::remove_cv<typename ::pdalboost::remove_reference<T>::type>::type))(v);
  }
//  template <typename T>
//  BOOST_THREAD_RV_REF(typename ::pdalboost::remove_cv<typename ::pdalboost::remove_reference<T>::type>::type)
//  make_rv_ref(T &v)  BOOST_NOEXCEPT
//  {
//    return (BOOST_THREAD_RV_REF(typename ::pdalboost::remove_cv<typename ::pdalboost::remove_reference<T>::type>::type))(v);
//  }
//  template <typename T>
//  const BOOST_THREAD_RV_REF(typename ::pdalboost::remove_cv<typename ::pdalboost::remove_reference<T>::type>::type)
//  make_rv_ref(T const&v)  BOOST_NOEXCEPT
//  {
//    return (const BOOST_THREAD_RV_REF(typename ::pdalboost::remove_cv<typename ::pdalboost::remove_reference<T>::type>::type))(v);
//  }
}
}

#define BOOST_THREAD_MAKE_RV_REF(RVALUE) RVALUE.move()
//#define BOOST_THREAD_MAKE_RV_REF(RVALUE) pdalboost::detail::make_rv_ref(RVALUE)
#endif


#if ! defined  BOOST_NO_CXX11_RVALUE_REFERENCES

#define BOOST_THREAD_MOVABLE(TYPE)

#else

#if defined BOOST_THREAD_USES_MOVE

#define BOOST_THREAD_MOVABLE(TYPE) \
    ::pdalboost::rv<TYPE>& move()  BOOST_NOEXCEPT \
    { \
      return *static_cast< ::pdalboost::rv<TYPE>* >(this); \
    } \
    const ::pdalboost::rv<TYPE>& move() const BOOST_NOEXCEPT \
    { \
      return *static_cast<const ::pdalboost::rv<TYPE>* >(this); \
    } \
    operator ::pdalboost::rv<TYPE>&() \
    { \
      return *static_cast< ::pdalboost::rv<TYPE>* >(this); \
    } \
    operator const ::pdalboost::rv<TYPE>&() const \
    { \
      return *static_cast<const ::pdalboost::rv<TYPE>* >(this); \
    }\

#else

#define BOOST_THREAD_MOVABLE(TYPE) \
    operator ::pdalboost::detail::thread_move_t<TYPE>() BOOST_NOEXCEPT \
    { \
        return move(); \
    } \
    ::pdalboost::detail::thread_move_t<TYPE> move() BOOST_NOEXCEPT \
    { \
      ::pdalboost::detail::thread_move_t<TYPE> x(*this); \
        return x; \
    } \

#endif
#endif

#define BOOST_THREAD_MOVABLE_ONLY(TYPE) \
  BOOST_THREAD_NO_COPYABLE(TYPE) \
  BOOST_THREAD_MOVABLE(TYPE) \

#define BOOST_THREAD_COPYABLE_AND_MOVABLE(TYPE) \
  BOOST_THREAD_MOVABLE(TYPE) \



namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost
{  namespace thread_detail
  {
#ifndef BOOST_NO_CXX11_RVALUE_REFERENCES
      template <class T>
      typename decay<T>::type
      decay_copy(T&& t)
      {
          return pdalboost::forward<T>(t);
      }
#else
  template <class T>
  typename decay<T>::type
  decay_copy(BOOST_THREAD_FWD_REF(T) t)
  {
      return pdalboost::forward<T>(t);
  }
#endif
  }
}

#include <boost/config/abi_suffix.hpp>

#endif
