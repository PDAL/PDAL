//
// detail/cstdint.hpp
// ~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef BOOST_ASIO_DETAIL_CSTDINT_HPP
#define BOOST_ASIO_DETAIL_CSTDINT_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <boost/asio/detail/config.hpp>

#if defined(BOOST_ASIO_HAS_CSTDINT)
# include <cstdint>
#else // defined(BOOST_ASIO_HAS_CSTDINT)
# include <boost/cstdint.hpp>
#endif // defined(BOOST_ASIO_HAS_CSTDINT)

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {
namespace asio {

#if defined(BOOST_ASIO_HAS_CSTDINT)
using std::int16_t;
using std::uint16_t;
using std::int32_t;
using std::uint32_t;
using std::int64_t;
using std::uint64_t;
#else // defined(BOOST_ASIO_HAS_CSTDINT)
using pdalboost::int16_t;
using pdalboost::uint16_t;
using pdalboost::int32_t;
using pdalboost::uint32_t;
using pdalboost::int64_t;
using pdalboost::uint64_t;
#endif // defined(BOOST_ASIO_HAS_CSTDINT)

} // namespace asio
} // namespace pdalboost

#endif // BOOST_ASIO_DETAIL_CSTDINT_HPP
