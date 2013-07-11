//
// detail/service_registry.hpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef BOOST_ASIO_DETAIL_SERVICE_REGISTRY_HPP
#define BOOST_ASIO_DETAIL_SERVICE_REGISTRY_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <boost/asio/detail/config.hpp>
#include <typeinfo>
#include <boost/asio/detail/mutex.hpp>
#include <boost/asio/detail/noncopyable.hpp>
#include <boost/asio/io_service.hpp>

#include <boost/asio/detail/push_options.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {
namespace asio {
namespace detail {

#if defined(__GNUC__)
# if (__GNUC__ == 4 && __GNUC_MINOR__ >= 1) || (__GNUC__ > 4)
#  pragma GCC visibility push (default)
# endif // (__GNUC__ == 4 && __GNUC_MINOR__ >= 1) || (__GNUC__ > 4)
#endif // defined(__GNUC__)

template <typename T>
class typeid_wrapper {};

#if defined(__GNUC__)
# if (__GNUC__ == 4 && __GNUC_MINOR__ >= 1) || (__GNUC__ > 4)
#  pragma GCC visibility pop
# endif // (__GNUC__ == 4 && __GNUC_MINOR__ >= 1) || (__GNUC__ > 4)
#endif // defined(__GNUC__)

class service_registry
  : private noncopyable
{
public:
  // Constructor. Adds the initial service.
  template <typename Service, typename Arg>
  service_registry(pdalboost::asio::io_service& o,
      Service* initial_service, Arg arg);

  // Destructor.
  BOOST_ASIO_DECL ~service_registry();

  // Notify all services of a fork event.
  BOOST_ASIO_DECL void notify_fork(pdalboost::asio::io_service::fork_event fork_ev);

  // Get the first service object cast to the specified type. Called during
  // io_service construction and so performs no locking or type checking.
  template <typename Service>
  Service& first_service();

  // Get the service object corresponding to the specified service type. Will
  // create a new service object automatically if no such object already
  // exists. Ownership of the service object is not transferred to the caller.
  template <typename Service>
  Service& use_service();

  // Add a service object. Throws on error, in which case ownership of the
  // object is retained by the caller.
  template <typename Service>
  void add_service(Service* new_service);

  // Check whether a service object of the specified type already exists.
  template <typename Service>
  bool has_service() const;

private:
  // Initialise a service's key based on its id.
  BOOST_ASIO_DECL static void init_key(
      pdalboost::asio::io_service::service::key& key,
      const pdalboost::asio::io_service::id& id);

#if !defined(BOOST_ASIO_NO_TYPEID)
  // Initialise a service's key based on its id.
  template <typename Service>
  static void init_key(pdalboost::asio::io_service::service::key& key,
      const pdalboost::asio::detail::service_id<Service>& /*id*/);
#endif // !defined(BOOST_ASIO_NO_TYPEID)

  // Check if a service matches the given id.
  BOOST_ASIO_DECL static bool keys_match(
      const pdalboost::asio::io_service::service::key& key1,
      const pdalboost::asio::io_service::service::key& key2);

  // The type of a factory function used for creating a service instance.
  typedef pdalboost::asio::io_service::service*
    (*factory_type)(pdalboost::asio::io_service&);

  // Factory function for creating a service instance.
  template <typename Service>
  static pdalboost::asio::io_service::service* create(
      pdalboost::asio::io_service& owner);

  // Destroy a service instance.
  BOOST_ASIO_DECL static void destroy(
      pdalboost::asio::io_service::service* service);

  // Helper class to manage service pointers.
  struct auto_service_ptr;
  friend struct auto_service_ptr;
  struct auto_service_ptr
  {
    pdalboost::asio::io_service::service* ptr_;
    ~auto_service_ptr() { destroy(ptr_); }
  };

  // Get the service object corresponding to the specified service key. Will
  // create a new service object automatically if no such object already
  // exists. Ownership of the service object is not transferred to the caller.
  BOOST_ASIO_DECL pdalboost::asio::io_service::service* do_use_service(
      const pdalboost::asio::io_service::service::key& key,
      factory_type factory);

  // Add a service object. Throws on error, in which case ownership of the
  // object is retained by the caller.
  BOOST_ASIO_DECL void do_add_service(
      const pdalboost::asio::io_service::service::key& key,
      pdalboost::asio::io_service::service* new_service);

  // Check whether a service object with the specified key already exists.
  BOOST_ASIO_DECL bool do_has_service(
      const pdalboost::asio::io_service::service::key& key) const;

  // Mutex to protect access to internal data.
  mutable pdalboost::asio::detail::mutex mutex_;

  // The owner of this service registry and the services it contains.
  pdalboost::asio::io_service& owner_;

  // The first service in the list of contained services.
  pdalboost::asio::io_service::service* first_service_;
};

} // namespace detail
} // namespace asio
} // namespace pdalboost

#include <boost/asio/detail/pop_options.hpp>

#include <boost/asio/detail/impl/service_registry.hpp>
#if defined(BOOST_ASIO_HEADER_ONLY)
# include <boost/asio/detail/impl/service_registry.ipp>
#endif // defined(BOOST_ASIO_HEADER_ONLY)

#endif // BOOST_ASIO_DETAIL_SERVICE_REGISTRY_HPP
