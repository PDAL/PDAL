//  operations_unit_test.cpp  ----------------------------------------------------------//

//  Copyright Beman Dawes 2008, 2009

//  Distributed under the Boost Software License, Version 1.0.
//  See http://www.boost.org/LICENSE_1_0.txt

//  Library home page: http://www.boost.org/libs/filesystem

//  ------------------------------------------------------------------------------------//

//  This program is misnamed - it is really a smoke test rather than a unit_test

//  ------------------------------------------------------------------------------------//

#define BOOST_FILESYSTEM_VERSION 3

#include <boost/config.hpp>

# if defined( BOOST_NO_STD_WSTRING )
#   error Configuration not supported: Boost.Filesystem V3 and later requires std::wstring support
# endif

#include <boost/config/warning_disable.hpp>

//  See deprecated_test for tests of deprecated features
#ifndef BOOST_FILESYSTEM_NO_DEPRECATED 
#  define BOOST_FILESYSTEM_NO_DEPRECATED
#endif
#ifndef BOOST_SYSTEM_NO_DEPRECATED 
#  define BOOST_SYSTEM_NO_DEPRECATED
#endif

#include <boost/filesystem.hpp>   // make sure filesystem.hpp works
#include <boost/system/error_code.hpp>
#include <boost/detail/lightweight_test.hpp>
#include <boost/detail/lightweight_main.hpp>
#include <iostream>

using namespace boost::filesystem;
using namespace boost::system;
using std::cout;
using std::string;

#define CHECK(x) check(x, __FILE__, __LINE__)

namespace
{

  void check(bool ok, const char* file, int line)
  {
    if (ok) return;

    ++::boost::detail::test_errors();

    std::cout << file << '(' << line << "): test failed\n";
  }

  //  query_test  ----------------------------------------------------------------------//

  void query_test()
  {
    std::cout << "query test..." << std::endl;

    error_code ec;

    CHECK(file_size("no-such-file", ec) == static_cast<boost::uintmax_t>(-1));
    CHECK(ec == errc::no_such_file_or_directory);

    CHECK(status("no-such-file") == file_status(file_not_found));

    CHECK(exists("/"));
    CHECK(is_directory("/"));
    CHECK(!exists("no-such-file"));

    exists("/", ec);
    if (ec)
    {
      std::cout << "exists(\"/\", ec) resulted in non-zero ec.value()" << std::endl;
      std::cout << "ec value: " << ec.value() << ", message: "<< ec.message() << std::endl;
    }
    CHECK(!ec);

    CHECK(exists("/"));
    CHECK(is_directory("/"));
    CHECK(!is_regular_file("/"));
    CHECK(!boost::filesystem::is_empty("/"));
    CHECK(!is_other("/"));
  }

  //  directory_iterator_test  -----------------------------------------------//

  void directory_iterator_test()
  {
    std::cout << "directory_iterator_test..." << std::endl;

    directory_iterator end;

    directory_iterator it(".");

    CHECK(!it->path().empty());

    if (is_regular_file(it->status()))
    {
      CHECK(is_regular_file(it->symlink_status()));
      CHECK(!is_directory(it->status()));
      CHECK(!is_symlink(it->status()));
      CHECK(!is_directory(it->symlink_status()));
      CHECK(!is_symlink(it->symlink_status()));
    }
    else
    {
      CHECK(is_directory(it->status()));
      CHECK(is_directory(it->symlink_status()));
      CHECK(!is_regular_file(it->status()));
      CHECK(!is_regular_file(it->symlink_status()));
      CHECK(!is_symlink(it->status()));
      CHECK(!is_symlink(it->symlink_status()));
    }

    for (; it != end; ++it)
    {
//      cout << "  " << it->path().string() << "\n";
    }

    std::cout << "directory_iterator_test complete" << std::endl;
  }

  //  operations_test  -------------------------------------------------------//

  void operations_test()
  {
    std::cout << "operations test..." << std::endl;

    error_code ec;

    CHECK(!create_directory("/", ec));

    CHECK(!boost::filesystem::remove("no-such-file-or-directory"));
    CHECK(!remove_all("no-such-file-or-directory"));

    space_info info = space("/");

    CHECK(info.available <= info.capacity);

    CHECK(equivalent("/", "/"));
    CHECK(!equivalent("/", "."));

    std::time_t ft = last_write_time(".");
    ft = -1;
    last_write_time(".", ft, ec);
  }

  //  directory_entry_test  ------------------------------------------------------------//

  void directory_entry_test()
  {
    std::cout << "directory_entry test..." << std::endl;

    directory_entry de("foo.bar", file_status(regular_file), file_status(directory_file));

    CHECK(de.path() == "foo.bar");
    CHECK(de.status() == file_status(regular_file));
    CHECK(de.symlink_status() == file_status(directory_file));
    CHECK(de < directory_entry("goo.bar"));
    CHECK(de == directory_entry("foo.bar"));
    CHECK(de != directory_entry("goo.bar"));
    de.replace_filename("bar.foo");
    CHECK(de.path() == "bar.foo");
  }

  //  directory_entry_overload_test  ---------------------------------------------------//

  void directory_entry_overload_test()
  {
    std::cout << "directory_entry overload test..." << std::endl;

    directory_iterator it(".");
    path p(*it);
  }

  //  error_handling_test  -------------------------------------------------------------//

  void error_handling_test()
  {
    std::cout << "error handling test..." << std::endl;

    bool threw(false);
    try
    { 
      file_size("no-such-file");
    }
    catch (const boost::filesystem::filesystem_error & ex)
    {
      threw = true;
      cout << "\nas expected, attempt to get size of non-existent file threw a filesystem_error\n"
        "what() returns " << ex.what() << "\n";
    }
    catch (...)
    {
      cout << "\nunexpected exception type caught" << std::endl;
    }

    CHECK(threw);

    error_code ec;
    CHECK(!create_directory("/", ec));
  }

}  // unnamed namespace

//--------------------------------------------------------------------------------------//
//                                                                                      //
//                                    main                                              //
//                                                                                      //
//--------------------------------------------------------------------------------------//

int cpp_main(int, char*[])
{
// document state of critical macros
#ifdef BOOST_POSIX_API
  cout << "BOOST_POSIX_API is defined\n";
#endif
#ifdef BOOST_WINDOWS_API
  cout << "BOOST_WINDOWS_API is defined\n";
#endif
  cout << "BOOST_FILESYSTEM_DECL" << BOOST_STRINGIZE(=BOOST_FILESYSTEM_DECL) << "\n";
  cout << "BOOST_SYMBOL_VISIBLE" << BOOST_STRINGIZE(=BOOST_SYMBOL_VISIBLE) << "\n";
  
  cout << "current_path() is " << current_path().string() << std::endl;

  query_test();
  directory_iterator_test();
  operations_test();
  directory_entry_test();
  directory_entry_overload_test();
  error_handling_test();

  std::cout << unique_path() << std::endl;
  std::cout << unique_path("foo-%%%%%-%%%%%-bar") << std::endl;
  
  return ::boost::report_errors();
}
