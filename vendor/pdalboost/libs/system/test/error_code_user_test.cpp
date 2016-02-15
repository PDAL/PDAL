//  error_code_user_test.cpp  ------------------------------------------------//

//  Copyright Beman Dawes 2006

//  Distributed under the Boost Software License, Version 1.0. (See accompanying
//  file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

//  See library home page at http://www.boost.org/libs/system

//  ------------------------------------------------------------------------  //

//  This program demonstrates creation and use of new categories of error
//  codes. Several scenarios are demonstrated and tested.

//  Motivation was a Boost posting by Christopher Kohlhoff on June 28, 2006.

#define BOOST_SYSTEM_NO_DEPRECATED

#include <boost/system/error_code.hpp>
#include <boost/cerrno.hpp>
#include <string>
#include <cstdio>
#include <boost/detail/lightweight_test.hpp>

#ifdef BOOST_POSIX_API
# include <sys/stat.h>
#else
# include <windows.h>
#endif

//  ------------------------------------------------------------------------  //

//  Library 1: User function passes through an error code from the
//  operating system. 


pdalboost::system::error_code my_mkdir( const std::string & path )
{
  return pdalboost::system::error_code(
#   ifdef BOOST_POSIX_API
      ::mkdir( path.c_str(), S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH ) == 0 ? 0 : errno,
#   else
      ::CreateDirectoryA( path.c_str(), 0 ) != 0 ? 0 : ::GetLastError(),
#   endif
      pdalboost::system::system_category() );
}

//  ------------------------------------------------------------------------  //

//  Library 2: User function passes through errno from the C-runtime. 

#include <cstdio>

pdalboost::system::error_code my_remove( const std::string & path )
{
  return pdalboost::system::error_code(
    std::remove( path.c_str() ) == 0 ? 0 : errno,
    pdalboost::system::generic_category() ); // OK for both Windows and POSIX
                                     // Alternatively, could use generic_category()
                                     // on Windows and system_category() on
                                     // POSIX-based systems.
}

//  ------------------------------------------------------------------------  //

//  Library 3: Library uses enum to identify library specific errors.

//  This particular example is for a library within the parent namespace. For
//  an example of a library not within the parent namespace, see library 4.

//  header lib3.hpp:

namespace pdalboost
{
  namespace lib3
  {
    // lib3 has its own error_category:
    const pdalboost::system::error_category & get_lib3_error_category() BOOST_SYSTEM_NOEXCEPT;
    const pdalboost::system::error_category & lib3_error_category = get_lib3_error_category();
    
    enum error
    {
      boo_boo=123,
      big_boo_boo
    };
  }

  namespace system
  {
    template<> struct is_error_code_enum<pdalboost::lib3::error>
      { static const bool value = true; };
  }

  namespace lib3
  {
    inline pdalboost::system::error_code make_error_code(error e)
      { return pdalboost::system::error_code(e,lib3_error_category); }
  }

}

//  implementation file lib3.cpp:

//  #include <lib3.hpp>

namespace pdalboost
{
  namespace lib3
  {
    class lib3_error_category_imp : public pdalboost::system::error_category
    {
    public:
      lib3_error_category_imp() : pdalboost::system::error_category() { }

      const char * name() const BOOST_SYSTEM_NOEXCEPT
      {
        return "lib3";
      }

      pdalboost::system::error_condition default_error_condition( int ev ) const BOOST_SYSTEM_NOEXCEPT
      {
        return ev == boo_boo
          ? pdalboost::system::error_condition( pdalboost::system::errc::io_error,
              pdalboost::system::generic_category() )
          : pdalboost::system::error_condition( ev,
              pdalboost::lib3::lib3_error_category );
      }
      
      std::string message( int ev ) const
      {
        if ( ev == boo_boo ) return std::string("boo boo");
        if ( ev == big_boo_boo ) return std::string("big boo boo");
        return std::string("unknown error");
      }

    };

    const pdalboost::system::error_category & get_lib3_error_category() BOOST_SYSTEM_NOEXCEPT
    {
      static const lib3_error_category_imp l3ecat;
      return l3ecat;
    }
  }
}

//  ------------------------------------------------------------------------  //

//  Library 4: Library uses const error_code's to identify library specific
//  errors. 

//  This particular example is for a library not within the parent namespace.
//  For an example of a library within the parent namespace, see library 3.

//  header lib4.hpp:

namespace lib4
{
  // lib4 has its own error_category:
  const pdalboost::system::error_category & get_lib4_error_category() BOOST_SYSTEM_NOEXCEPT;
  const pdalboost::system::error_category & lib4_error_category = get_lib4_error_category();
  
  extern const pdalboost::system::error_code boo_boo;
  extern const pdalboost::system::error_code big_boo_boo;
}

//  implementation file lib4.cpp:

//  #include <lib4.hpp>

namespace lib4
{
  class lib4_error_category_imp : public pdalboost::system::error_category
  {
  public:
    lib4_error_category_imp() : pdalboost::system::error_category() { }

    const char * name() const BOOST_SYSTEM_NOEXCEPT
    {
      return "lib4";
    }

    pdalboost::system::error_condition default_error_condition( int ev ) const  BOOST_SYSTEM_NOEXCEPT
    {
      return ev == boo_boo.value()
        ? pdalboost::system::error_condition( pdalboost::system::errc::io_error,
            pdalboost::system::generic_category() )
        : pdalboost::system::error_condition( ev, lib4::lib4_error_category );
    }
    
    std::string message( int ev ) const
    {
      if ( ev == boo_boo.value() ) return std::string("boo boo");
      if ( ev == big_boo_boo.value() ) return std::string("big boo boo");
      return std::string("unknown error");
    }
  };

  const pdalboost::system::error_category & get_lib4_error_category() BOOST_SYSTEM_NOEXCEPT
  {
    static const lib4_error_category_imp l4ecat;
    return l4ecat;
  }

  const pdalboost::system::error_code boo_boo( 456, lib4_error_category );
  const pdalboost::system::error_code big_boo_boo( 789, lib4_error_category );

}

//  ------------------------------------------------------------------------  //

// Chris Kolhoff's Test3, modified to work with error_code.hpp

// Test3
// =====
// Define error classes to check for success, permission_denied and
// out_of_memory, but add additional mappings for a user-defined error category.
//

//namespace test3 {

//  enum user_err
//  {
//    user_success = 0,
//    user_permission_denied,
//    user_out_of_memory
//  };
//
//  class user_error_category_imp : public pdalboost::system::error_category
//  {
//  public:
//    const std::string & name() const
//    {
//      static std::string s( "test3" );
//      return s;
//    }
//
//    pdalboost::system::error_code portable_error_code( int ev ) const
//    {
//      switch (ev)
//      {
//        case user_success:
//          return pdalboost::system::error_code(pdalboost::system::errc::success, pdalboost::system::generic_category());
//        case user_permission_denied:
//          return pdalboost::system::error_code(pdalboost::system::errc::permission_denied, pdalboost::system::generic_category());
//        case user_out_of_memory:
//          return pdalboost::system::error_code(pdalboost::system::errc::not_enough_memory, pdalboost::system::generic_category());
//        default:
//          break;
//      }
//      return pdalboost::system::error_code(pdalboost::system::errc::no_posix_equivalent, pdalboost::system::generic_category());
//    }
//    
//  };
//
//  const user_error_category_imp user_error_category_const;
//
//  const pdalboost::system::error_category & user_error_category
//    = user_error_category_const;
//
//  template<> inline pdalboost::system::error_code make_error_code(user_err e)
//  {
//    return pdalboost::system::error_code(e, user_error_category);
//  }
//
//  // test code
//
//  void check_success(const pdalboost::system::error_code& ec, bool expect)
//  {
//    BOOST_TEST( (ec == pdalboost::system::errc::success) == expect );
//    if (ec == pdalboost::system::errc::success)
//      std::cout << "yes... " << (expect ? "ok" : "fail") << '\n';
//    else
//      std::cout << "no...  " << (expect ? "fail" : "ok") << '\n';
//  }
//
//  void check_permission_denied(const pdalboost::system::error_code& ec, bool expect)
//  {
//    BOOST_TEST( (ec == pdalboost::system::errc::permission_denied) == expect );
//    if (ec ==  pdalboost::system::errc::permission_denied)
//      std::cout << "yes... " << (expect ? "ok" : "fail") << '\n';
//    else
//      std::cout << "no...  " << (expect ? "fail" : "ok") << '\n';
//  }
//
//  void check_out_of_memory(const pdalboost::system::error_code& ec, bool expect)
//  {
//    BOOST_TEST( (ec == pdalboost::system::errc::not_enough_memory) == expect );
//    if (ec ==  pdalboost::system::errc::not_enough_memory)
//      std::cout << "yes... " << (expect ? "ok" : "fail") << '\n';
//    else
//      std::cout << "no...  " << (expect ? "fail" : "ok") << '\n';
//  }
//
//  void run()
//  {
//    printf("Test3\n");
//    printf("=====\n");
//    pdalboost::system::error_code ec;
//    check_success(ec, true);
//    check_success(pdalboost::system::errc::success, true);
//    check_success(pdalboost::system::errc::permission_denied, false);
//    check_success(pdalboost::system::errc::not_enough_memory, false);
//    check_success(user_success, true);
//    check_success(user_permission_denied, false);
//    check_success(user_out_of_memory, false);
//    check_permission_denied(ec, false);
//    check_permission_denied(pdalboost::system::errc::success, false);
//    check_permission_denied(pdalboost::system::errc::permission_denied, true);
//    check_permission_denied(pdalboost::system::errc::not_enough_memory, false);
//    check_permission_denied(user_success, false);
//    check_permission_denied(user_permission_denied, true);
//    check_permission_denied(user_out_of_memory, false);
//    check_out_of_memory(ec, false);
//    check_out_of_memory(pdalboost::system::errc::success, false);
//    check_out_of_memory(pdalboost::system::errc::permission_denied, false);
//    check_out_of_memory(pdalboost::system::errc::not_enough_memory, true);
//    check_out_of_memory(user_success, false);
//    check_out_of_memory(user_permission_denied, false);
//    check_out_of_memory(user_out_of_memory, true);
//
//# ifdef BOOST_WINDOWS_API
//    check_success(pdalboost::system::windows::success, true);
//    check_success(pdalboost::system::windows::access_denied, false);
//    check_success(pdalboost::system::windows::not_enough_memory, false);
//    check_permission_denied(pdalboost::system::windows::success, false);
//    check_permission_denied(pdalboost::system::windows::access_denied, true);
//    check_permission_denied(pdalboost::system::windows::not_enough_memory, false);
//    check_out_of_memory(pdalboost::system::windows::success, false);
//    check_out_of_memory(pdalboost::system::windows::access_denied, false);
//    check_out_of_memory(pdalboost::system::windows::not_enough_memory, true);
//# endif
//
//    printf("\n");
//  }
//
//} // namespace test3



//  ------------------------------------------------------------------------  //

int main( int, char *[] )
{
  pdalboost::system::error_code ec;

  // Library 1 tests:
  
  ec = my_mkdir( "/no-such-file-or-directory/will-not-succeed" );
  std::cout << "ec.value() is " << ec.value() << '\n';

  BOOST_TEST( ec );
  BOOST_TEST( ec == pdalboost::system::errc::no_such_file_or_directory );
  BOOST_TEST( ec.category() == pdalboost::system::system_category() );

  // Library 2 tests:

  ec = my_remove( "/no-such-file-or-directory" );
  std::cout << "ec.value() is " << ec.value() << '\n';

  BOOST_TEST( ec );
  BOOST_TEST( ec == pdalboost::system::errc::no_such_file_or_directory );
  BOOST_TEST( ec.category() == pdalboost::system::generic_category() );

  // Library 3 tests:

  ec = pdalboost::lib3::boo_boo;
  std::cout << "ec.value() is " << ec.value() << '\n';

  BOOST_TEST( ec );
  BOOST_TEST( ec == pdalboost::lib3::boo_boo );
  BOOST_TEST( ec.value() == pdalboost::lib3::boo_boo );
  BOOST_TEST( ec.category() == pdalboost::lib3::lib3_error_category );

  BOOST_TEST( ec == pdalboost::system::errc::io_error );

  pdalboost::system::error_code ec3( pdalboost::lib3::boo_boo+100,
    pdalboost::lib3::lib3_error_category );
  BOOST_TEST( ec3.category() == pdalboost::lib3::lib3_error_category );
  BOOST_TEST( ec3.default_error_condition().category()
    == pdalboost::lib3::lib3_error_category );

  // Library 4 tests:

  ec = lib4::boo_boo;
  std::cout << "ec.value() is " << ec.value() << '\n';

  BOOST_TEST( ec );
  BOOST_TEST( ec == lib4::boo_boo );
  BOOST_TEST( ec.value() == lib4::boo_boo.value() );
  BOOST_TEST( ec.category() == lib4::lib4_error_category );

  BOOST_TEST( ec == pdalboost::system::errc::io_error );

  pdalboost::system::error_code ec4( lib4::boo_boo.value()+100,
    lib4::lib4_error_category );
  BOOST_TEST( ec4.default_error_condition().category()
    == lib4::lib4_error_category );

  // Test 3

  //test3::run();

  return ::pdalboost::report_errors();
}
