//
// ssl/old/context_service.hpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2005 Voipster / Indrek dot Juhani at voipster dot com
// Copyright (c) 2005-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef BOOST_ASIO_SSL_OLD_CONTEXT_SERVICE_HPP
#define BOOST_ASIO_SSL_OLD_CONTEXT_SERVICE_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <boost/asio/detail/config.hpp>
#include <string>
#include <boost/noncopyable.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ssl/context_base.hpp>
#include <boost/asio/ssl/old/detail/openssl_context_service.hpp>

#include <boost/asio/detail/push_options.hpp>

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost {
namespace asio {
namespace ssl {
namespace old {

/// Default service implementation for a context.
class context_service
#if defined(GENERATING_DOCUMENTATION)
  : public pdalboost::asio::io_service::service
#else
  : public pdalboost::asio::detail::service_base<context_service>
#endif
{
private:
  // The type of the platform-specific implementation.
  typedef old::detail::openssl_context_service service_impl_type;

public:
#if defined(GENERATING_DOCUMENTATION)
  /// The unique service identifier.
  static pdalboost::asio::io_service::id id;
#endif

  /// The type of the context.
#if defined(GENERATING_DOCUMENTATION)
  typedef implementation_defined impl_type;
#else
  typedef service_impl_type::impl_type impl_type;
#endif

  /// Constructor.
  explicit context_service(pdalboost::asio::io_service& io_service)
    : pdalboost::asio::detail::service_base<context_service>(io_service),
      service_impl_(pdalboost::asio::use_service<service_impl_type>(io_service))
  {
  }

  /// Return a null context implementation.
  impl_type null() const
  {
    return service_impl_.null();
  }

  /// Create a new context implementation.
  void create(impl_type& impl, context_base::method m)
  {
    service_impl_.create(impl, m);
  }

  /// Destroy a context implementation.
  void destroy(impl_type& impl)
  {
    service_impl_.destroy(impl);
  }

  /// Set options on the context.
  pdalboost::system::error_code set_options(impl_type& impl,
      context_base::options o, pdalboost::system::error_code& ec)
  {
    return service_impl_.set_options(impl, o, ec);
  }

  /// Set peer verification mode.
  pdalboost::system::error_code set_verify_mode(impl_type& impl,
      context_base::verify_mode v, pdalboost::system::error_code& ec)
  {
    return service_impl_.set_verify_mode(impl, v, ec);
  }

  /// Load a certification authority file for performing verification.
  pdalboost::system::error_code load_verify_file(impl_type& impl,
      const std::string& filename, pdalboost::system::error_code& ec)
  {
    return service_impl_.load_verify_file(impl, filename, ec);
  }

  /// Add a directory containing certification authority files to be used for
  /// performing verification.
  pdalboost::system::error_code add_verify_path(impl_type& impl,
      const std::string& path, pdalboost::system::error_code& ec)
  {
    return service_impl_.add_verify_path(impl, path, ec);
  }

  /// Use a certificate from a file.
  pdalboost::system::error_code use_certificate_file(impl_type& impl,
      const std::string& filename, context_base::file_format format,
      pdalboost::system::error_code& ec)
  {
    return service_impl_.use_certificate_file(impl, filename, format, ec);
  }

  /// Use a certificate chain from a file.
  pdalboost::system::error_code use_certificate_chain_file(impl_type& impl,
      const std::string& filename, pdalboost::system::error_code& ec)
  {
    return service_impl_.use_certificate_chain_file(impl, filename, ec);
  }

  /// Use a private key from a file.
  pdalboost::system::error_code use_private_key_file(impl_type& impl,
      const std::string& filename, context_base::file_format format,
      pdalboost::system::error_code& ec)
  {
    return service_impl_.use_private_key_file(impl, filename, format, ec);
  }

  /// Use an RSA private key from a file.
  pdalboost::system::error_code use_rsa_private_key_file(impl_type& impl,
      const std::string& filename, context_base::file_format format,
      pdalboost::system::error_code& ec)
  {
    return service_impl_.use_rsa_private_key_file(impl, filename, format, ec);
  }

  /// Use the specified file to obtain the temporary Diffie-Hellman parameters.
  pdalboost::system::error_code use_tmp_dh_file(impl_type& impl,
      const std::string& filename, pdalboost::system::error_code& ec)
  {
    return service_impl_.use_tmp_dh_file(impl, filename, ec);
  }

  /// Set the password callback.
  template <typename PasswordCallback>
  pdalboost::system::error_code set_password_callback(impl_type& impl,
      PasswordCallback callback, pdalboost::system::error_code& ec)
  {
    return service_impl_.set_password_callback(impl, callback, ec);
  }

private:
  // Destroy all user-defined handler objects owned by the service.
  void shutdown_service()
  {
  }

  // The service that provides the platform-specific implementation.
  service_impl_type& service_impl_;
};

} // namespace old
} // namespace ssl
} // namespace asio
} // namespace pdalboost

#include <boost/asio/detail/pop_options.hpp>

#endif // BOOST_ASIO_SSL_OLD_CONTEXT_SERVICE_HPP
