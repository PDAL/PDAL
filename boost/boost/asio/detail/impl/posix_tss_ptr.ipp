//
// detail/impl/posix_tss_ptr.ipp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef BOOST_ASIO_DETAIL_IMPL_POSIX_TSS_PTR_IPP
#define BOOST_ASIO_DETAIL_IMPL_POSIX_TSS_PTR_IPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <boost/asio/detail/config.hpp>

#if defined(BOOST_ASIO_HAS_PTHREADS)

#include <boost/asio/detail/posix_tss_ptr.hpp>
#include <boost/asio/detail/throw_error.hpp>
#include <boost/asio/error.hpp>

#include <boost/asio/detail/push_options.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {
namespace asio {
namespace detail {

void posix_tss_ptr_create(pthread_key_t& key)
{
  int error = ::pthread_key_create(&key, 0);
  pdalboost::system::error_code ec(error,
      pdalboost::asio::error::get_system_category());
  pdalboost::asio::detail::throw_error(ec, "tss");
}

} // namespace detail
} // namespace asio
} // namespace pdalboost

#include <boost/asio/detail/pop_options.hpp>

#endif // defined(BOOST_ASIO_HAS_PTHREADS)

#endif // BOOST_ASIO_DETAIL_IMPL_POSIX_TSS_PTR_IPP
