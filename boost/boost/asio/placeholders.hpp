//
// placeholders.hpp
// ~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef BOOST_ASIO_PLACEHOLDERS_HPP
#define BOOST_ASIO_PLACEHOLDERS_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <boost/asio/detail/config.hpp>

#if defined(BOOST_ASIO_HAS_BOOST_BIND)
# include <boost/bind/arg.hpp>
#endif // defined(BOOST_ASIO_HAS_BOOST_BIND)

#include <boost/asio/detail/push_options.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {
namespace asio {
namespace placeholders {

#if defined(GENERATING_DOCUMENTATION)

/// An argument placeholder, for use with pdalboost::bind(), that corresponds to
/// the error argument of a handler for any of the asynchronous functions.
unspecified error;

/// An argument placeholder, for use with pdalboost::bind(), that corresponds to
/// the bytes_transferred argument of a handler for asynchronous functions such
/// as pdalboost::asio::basic_stream_socket::async_write_some or
/// pdalboost::asio::async_write.
unspecified bytes_transferred;

/// An argument placeholder, for use with pdalboost::bind(), that corresponds to
/// the iterator argument of a handler for asynchronous functions such as
/// pdalboost::asio::basic_resolver::async_resolve.
unspecified iterator;

/// An argument placeholder, for use with pdalboost::bind(), that corresponds to
/// the signal_number argument of a handler for asynchronous functions such as
/// pdalboost::asio::signal_set::async_wait.
unspecified signal_number;

#elif defined(BOOST_ASIO_HAS_BOOST_BIND)
# if defined(__BORLANDC__) || defined(__GNUC__)

inline pdalboost::arg<1> error()
{
  return pdalboost::arg<1>();
}

inline pdalboost::arg<2> bytes_transferred()
{
  return pdalboost::arg<2>();
}

inline pdalboost::arg<2> iterator()
{
  return pdalboost::arg<2>();
}

inline pdalboost::arg<2> signal_number()
{
  return pdalboost::arg<2>();
}

# else

namespace detail
{
  template <int Number>
  struct placeholder
  {
    static pdalboost::arg<Number>& get()
    {
      static pdalboost::arg<Number> result;
      return result;
    }
  };
}

#  if defined(BOOST_ASIO_MSVC) && (BOOST_ASIO_MSVC < 1400)

static pdalboost::arg<1>& error
  = pdalboost::asio::placeholders::detail::placeholder<1>::get();
static pdalboost::arg<2>& bytes_transferred
  = pdalboost::asio::placeholders::detail::placeholder<2>::get();
static pdalboost::arg<2>& iterator
  = pdalboost::asio::placeholders::detail::placeholder<2>::get();
static pdalboost::arg<2>& signal_number
  = pdalboost::asio::placeholders::detail::placeholder<2>::get();

#  else

namespace
{
  pdalboost::arg<1>& error
    = pdalboost::asio::placeholders::detail::placeholder<1>::get();
  pdalboost::arg<2>& bytes_transferred
    = pdalboost::asio::placeholders::detail::placeholder<2>::get();
  pdalboost::arg<2>& iterator
    = pdalboost::asio::placeholders::detail::placeholder<2>::get();
  pdalboost::arg<2>& signal_number
    = pdalboost::asio::placeholders::detail::placeholder<2>::get();
} // namespace

#  endif
# endif
#endif

} // namespace placeholders
} // namespace asio
} // namespace pdalboost

#include <boost/asio/detail/pop_options.hpp>

#endif // BOOST_ASIO_PLACEHOLDERS_HPP
