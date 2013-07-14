//
// ssl/error.hpp
// ~~~~~~~~~~~~~
//
// Copyright (c) 2003-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef BOOST_ASIO_SSL_ERROR_HPP
#define BOOST_ASIO_SSL_ERROR_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <boost/asio/detail/config.hpp>
#include <boost/system/error_code.hpp>

#include <boost/asio/detail/push_options.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {
namespace asio {
namespace error {

enum ssl_errors
{
};

extern BOOST_ASIO_DECL
const pdalboost::system::error_category& get_ssl_category();

static const pdalboost::system::error_category& ssl_category
  = pdalboost::asio::error::get_ssl_category();

} // namespace error
} // namespace asio
} // namespace pdalboost

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {
namespace system {

template<> struct is_error_code_enum<pdalboost::asio::error::ssl_errors>
{
  static const bool value = true;
};

} // namespace system
} // namespace pdalboost

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {
namespace asio {
namespace error {

inline pdalboost::system::error_code make_error_code(ssl_errors e)
{
  return pdalboost::system::error_code(
      static_cast<int>(e), get_ssl_category());
}

} // namespace error
} // namespace asio
} // namespace pdalboost

#include <boost/asio/detail/pop_options.hpp>

#if defined(BOOST_ASIO_HEADER_ONLY)
# include <boost/asio/ssl/impl/error.ipp>
#endif // defined(BOOST_ASIO_HEADER_ONLY)

#endif // BOOST_ASIO_SSL_ERROR_HPP
