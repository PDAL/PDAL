// Copyright (C) 2012
// Vicente J. Botet Escriba
//
//  Distributed under the Boost Software License, Version 1.0. (See accompanying
//  file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_THREAD_DETAIL_SCOPED_ENUM_HPP
#define BOOST_THREAD_DETAIL_SCOPED_ENUM_HPP

#include <boost/config.hpp>
#include <boost/detail/workaround.hpp>

namespace boost
{

#ifdef BOOST_NO_SCOPED_ENUMS
  template <typename NT>
  struct underlying_type
  {
    typedef typename NT::underlying_type type;
  };

  template <typename UT, typename NT>
  UT underlying_cast(NT v)
  {
    return v.underlying();
  }

  template <typename EC>
  inline
  typename EC::enum_type native_value(EC e)
  {
    return e.native();
  }

#else  // BOOST_NO_SCOPED_ENUMS

  template <typename NT>
  struct underlying_type
  {
    //typedef typename std::underlying_type<NT>::type type;
  };

  template <typename UT, typename NT>
  UT underlying_cast(NT v)
  {
    return static_cast<UT>(v);
  }

  template <typename EC>
  inline
  EC native_value(EC e)
  {
    return e;
 }

#endif
}


#ifdef BOOST_NO_SCOPED_ENUMS

#ifndef BOOST_NO_EXPLICIT_CONVERSION_OPERATORS

#define BOOST_SCOPED_ENUM_UT_DECLARE_CONVERSION_OPERATOR \
     explicit operator underlying_type() const { return underlying(); }

#else

#define BOOST_SCOPED_ENUM_UT_DECLARE_CONVERSION_OPERATOR

#endif

#define BOOST_SCOPED_ENUM_UT_DECLARE_BEGIN(NT, UT) \
  struct NT { \
    typedef UT underlying_type; \
    enum enum_type

#define BOOST_SCOPED_ENUM_DECLARE_END(NT) \
    ; \
    NT()  {} \
    NT(enum_type v) : v_(v) {} \
    explicit NT(underlying_type v) : v_(v) {} \
    underlying_type underlying() const { return v_; } \
    enum_type native() const { return enum_type(v_); } \
    BOOST_SCOPED_ENUM_UT_DECLARE_CONVERSION_OPERATOR \
    friend bool operator ==(NT lhs, enum_type rhs)  { return enum_type(lhs.v_)==rhs; } \
    friend bool operator ==(enum_type lhs, NT rhs)  { return lhs==enum_type(rhs.v_); } \
    friend bool operator !=(NT lhs, enum_type rhs)  { return enum_type(lhs.v_)!=rhs; } \
    friend bool operator !=(enum_type lhs, NT rhs)  { return lhs!=enum_type(rhs.v_); } \
  private: \
    underlying_type v_; \
  };

#define BOOST_SCOPED_ENUM_DECLARE_BEGIN(NT) \
  BOOST_SCOPED_ENUM_UT_DECLARE_BEGIN(NT,int)

#define BOOST_SCOPED_ENUM_NATIVE(NT) NT::enum_type
#define BOOST_SCOPED_ENUM_FORWARD_DECLARE(NT) struct NT

#else  // BOOST_NO_SCOPED_ENUMS

#define BOOST_SCOPED_ENUM_UT_DECLARE_BEGIN(NT,UT) enum class NT:UT
#define BOOST_SCOPED_ENUM_DECLARE_BEGIN(NT) enum class NT
#define BOOST_SCOPED_ENUM_DECLARE_END(NT) ;

#define BOOST_SCOPED_ENUM_NATIVE(NT) NT
#define BOOST_SCOPED_ENUM_FORWARD_DECLARE(NT) enum class NT

#endif  // BOOST_NO_SCOPED_ENUMS


#endif // BOOST_THREAD_DETAIL_SCOPED_ENUM_HPP
