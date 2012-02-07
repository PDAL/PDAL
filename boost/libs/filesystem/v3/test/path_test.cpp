//  path_test program  -----------------------------------------------------------------//

//  Copyright Beman Dawes 2002, 2008
//  Copyright Vladimir Prus 2002

//  Use, modification, and distribution is subject to the Boost Software
//  License, Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)

//  See library home page at http://www.boost.org/libs/filesystem

//  basic_path's stem(), extension(), and replace_extension() tests are based
//  on basename(), extension(), and change_extension() tests from the original
//  convenience_test.cpp by Vladimir Prus.

//--------------------------------------------------------------------------------------//
//                                                                                      //
//                                     Caution                                          //
//                                                                                      //
//  The class path relational operators (==, !=, <, etc.) on Windows treat slash and    //
//  backslash as equal. Thus any tests on Windows where the difference between slash    //
//  and backslash is significant should compare based on native observers rather than   //
//  directly on path objects.                                                           //
//                                                                                      //
//--------------------------------------------------------------------------------------//

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

#include <boost/filesystem/operations.hpp>
#include <boost/utility.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <cassert>

namespace fs = boost::filesystem;
using boost::filesystem::path;

#include <boost/detail/lightweight_test.hpp>
#include <boost/detail/lightweight_main.hpp>

#define PATH_CHECK(a, b) check(a, b, __FILE__, __LINE__)
#define CHECK_EQUAL(a,b) check_equal(a, b, __FILE__, __LINE__)

namespace
{
  std::string platform(BOOST_PLATFORM);

  void check(const fs::path & source,
              const std::string & expected, const char* file, int line)
  {
    if (source.string() == expected)
      return;

    std::cout << file
              << '(' << line << "): source: \"" << source
              << "\" != expected: \"" << expected
              << "\"" << std::endl;

    ++::boost::detail::test_errors();
  }

  void check_equal(const fs::path & source,
              const std::string & expected, const char* file, int line)
  {
    if (source == expected) return;

    ++::boost::detail::test_errors();

    std::cout << file << '(' << line << "): source: \"" << source
              << "\" != expected: \"" << expected
              << "\"" << std::endl;
  }

  path p1("fe/fi/fo/fum");
  path p2(p1);
  path p3;
  path p4("foobar");
  path p5;

  //  exception_tests  -----------------------------------------------------------------//

  void exception_tests()
  {
    std::cout << "exception_tests..." << std::endl;
    const std::string str_1("string-1");
    boost::system::error_code ec(12345, boost::system::system_category());
    try { throw fs::filesystem_error(str_1, ec); }
    catch (const fs::filesystem_error & ex)
    {
      //std::cout << ex.what() << "*" << std::endl;
      //BOOST_TEST(std::strcmp(ex.what(),
      //  "string-1: Unknown error") == 0);
      BOOST_TEST(ex.code() == ec);
    }

    try { throw fs::filesystem_error(str_1, "p1", "p2", ec); }
    catch (const fs::filesystem_error & ex)
    {
      //std::cout << ex.what() << "*" << std::endl;                    
      //BOOST_TEST(std::strcmp(ex.what(),
      //  "string-1: Unknown error: \"p1\", \"p2\"") == 0);
      BOOST_TEST(ex.code() == ec);
      BOOST_TEST(ex.path1() == "p1");
      BOOST_TEST(ex.path2() == "p2");
    }
  }

  //  overload_tests  ------------------------------------------------------------------//

  // These verify various overloads don't cause compiler errors
  // They pre-date operations_unit_test.cpp

  void overload_tests()
  {
    std::cout << "overload_tests..." << std::endl;

    fs::exists(p1);
    fs::exists("foo");
    fs::exists(std::string("foo"));

    fs::exists(p1 / path("foo"));
    fs::exists(p1 / "foo");
    fs::exists(p1 / std::string("foo"));

    fs::exists("foo" / p1);
    fs::exists(std::string("foo") / p1);

    p4 /= path("foo");
    p4 /= "foo";
    p4 /= std::string("foo");
  }

  //  iterator_tests  ------------------------------------------------------------------//

  void iterator_tests()
  {
    std::cout << "iterator_tests..." << std::endl;

    path itr_ck = "";
    path::const_iterator itr = itr_ck.begin();
    BOOST_TEST(itr == itr_ck.end());

    itr_ck = "/";
    itr = itr_ck.begin();
    BOOST_TEST(*itr == std::string("/"));
    BOOST_TEST(++itr == itr_ck.end());
    BOOST_TEST(*--itr == std::string("/"));

    itr_ck = "foo";
    BOOST_TEST(*itr_ck.begin() == std::string("foo"));
    BOOST_TEST(boost::next(itr_ck.begin()) == itr_ck.end());
    BOOST_TEST(*boost::prior(itr_ck.end()) == std::string("foo"));
    BOOST_TEST(boost::prior(itr_ck.end()) == itr_ck.begin());

    itr_ck = path("/foo");
    BOOST_TEST(*itr_ck.begin() == std::string("/"));
    BOOST_TEST(*boost::next(itr_ck.begin()) == std::string("foo"));
    BOOST_TEST(boost::next(boost::next(itr_ck.begin())) == itr_ck.end());
    BOOST_TEST(boost::next(itr_ck.begin()) == boost::prior(itr_ck.end()));
    BOOST_TEST(*boost::prior(itr_ck.end()) == std::string("foo"));
    BOOST_TEST(*boost::prior(boost::prior(itr_ck.end())) == std::string("/"));
    BOOST_TEST(boost::prior(boost::prior(itr_ck.end())) == itr_ck.begin());

    itr_ck = "/foo/bar";
    itr = itr_ck.begin();
    BOOST_TEST(*itr == std::string("/"));
    BOOST_TEST(*++itr == std::string("foo"));
    BOOST_TEST(*++itr == std::string("bar"));
    BOOST_TEST(++itr == itr_ck.end());
    CHECK_EQUAL(*--itr, "bar");
    CHECK_EQUAL(*--itr, "foo");
    CHECK_EQUAL(*--itr, "/");

    itr_ck = "../f"; // previously failed due to short name bug
    itr = itr_ck.begin();
    CHECK_EQUAL(*itr, "..");
    CHECK_EQUAL(*++itr, "f");
    BOOST_TEST(++itr == itr_ck.end());
    CHECK_EQUAL(*--itr, "f");
    CHECK_EQUAL(*--itr, "..");

    // POSIX says treat "/foo/bar/" as "/foo/bar/."
    itr_ck = "/foo/bar/";
    itr = itr_ck.begin();
    CHECK_EQUAL(*itr, "/");
    CHECK_EQUAL(*++itr, "foo");
    CHECK_EQUAL(*++itr, "bar");
    CHECK_EQUAL(*++itr, ".");
    BOOST_TEST(++itr == itr_ck.end());
    CHECK_EQUAL(*--itr, ".");
    CHECK_EQUAL(*--itr, "bar");
    CHECK_EQUAL(*--itr, "foo");
    CHECK_EQUAL(*--itr, "/");

    // POSIX says treat "/f/b/" as "/f/b/."
    itr_ck = "/f/b/";
    itr = itr_ck.begin();
    CHECK_EQUAL(*itr, "/");
    CHECK_EQUAL(*++itr, "f");
    CHECK_EQUAL(*++itr, "b");
    CHECK_EQUAL(*++itr, ".");
    BOOST_TEST(++itr == itr_ck.end());
    CHECK_EQUAL(*--itr, ".");
    CHECK_EQUAL(*--itr, "b");
    CHECK_EQUAL(*--itr, "f");
    CHECK_EQUAL(*--itr, "/");

    itr_ck = "//net";
    itr = itr_ck.begin();
    // two leading slashes are permitted by POSIX (as implementation defined),
    // while for Windows it is always well defined (as a network name)
    CHECK_EQUAL(*itr, "//net");
    BOOST_TEST(++itr == itr_ck.end());
    CHECK_EQUAL(*--itr, "//net");

    itr_ck = "//net/";
    itr = itr_ck.begin();
    CHECK_EQUAL(*itr, "//net");
    CHECK_EQUAL(*++itr, "/");
    BOOST_TEST(++itr == itr_ck.end());
    CHECK_EQUAL(*--itr, "/");
    CHECK_EQUAL(*--itr, "//net");

    itr_ck = "//foo///bar///";
    itr = itr_ck.begin();
    CHECK_EQUAL(*itr, "//foo");
    CHECK_EQUAL(*++itr, "/");
    CHECK_EQUAL(*++itr, "bar");
    CHECK_EQUAL(*++itr, ".");
    BOOST_TEST(++itr == itr_ck.end());
    CHECK_EQUAL(*--itr, ".");
    CHECK_EQUAL(*--itr, "bar");
    CHECK_EQUAL(*--itr, "/");
    CHECK_EQUAL(*--itr, "//foo");

    itr_ck = "///foo///bar///";
    itr = itr_ck.begin();
    // three or more leading slashes are to be treated as a single slash
    CHECK_EQUAL(*itr, "/");
    CHECK_EQUAL(*++itr, "foo");
    CHECK_EQUAL(*++itr, "bar");
    CHECK_EQUAL(*++itr, ".");
    BOOST_TEST(++itr == itr_ck.end());
    CHECK_EQUAL(*--itr, ".");
    CHECK_EQUAL(*--itr, "bar");
    CHECK_EQUAL(*--itr, "foo");
    CHECK_EQUAL(*--itr, "/");

    if (platform == "Windows")
    {
      itr_ck = "c:/";
      itr = itr_ck.begin();
      CHECK_EQUAL(*itr, "c:");
      CHECK_EQUAL(*++itr, "/");
      BOOST_TEST(++itr == itr_ck.end());
      CHECK_EQUAL(*--itr, "/");
      CHECK_EQUAL(*--itr, "c:");

      itr_ck = "c:\\foo";
      itr = itr_ck.begin();
      BOOST_TEST(*itr == std::string("c:"));
      BOOST_TEST(*++itr == std::string("/"));
      BOOST_TEST(*++itr == std::string("foo"));
      BOOST_TEST(++itr == itr_ck.end());
      BOOST_TEST(*--itr == std::string("foo"));
      BOOST_TEST(*--itr == std::string("/"));
      BOOST_TEST(*--itr == std::string("c:"));

      itr_ck = "\\\\\\foo\\\\\\bar\\\\\\";
      itr = itr_ck.begin();
      // three or more leading slashes are to be treated as a single slash
      CHECK_EQUAL(*itr, "/");
      CHECK_EQUAL(*++itr, "foo");
      CHECK_EQUAL(*++itr, "bar");
      CHECK_EQUAL(*++itr, ".");
      BOOST_TEST(++itr == itr_ck.end());
      CHECK_EQUAL(*--itr, ".");
      CHECK_EQUAL(*--itr, "bar");
      CHECK_EQUAL(*--itr, "foo");
      CHECK_EQUAL(*--itr, "/");

      itr_ck = "c:foo";
      itr = itr_ck.begin();
      BOOST_TEST(*itr == std::string("c:"));
      BOOST_TEST(*++itr == std::string("foo"));
      BOOST_TEST(++itr == itr_ck.end());
      BOOST_TEST(*--itr == std::string("foo"));
      BOOST_TEST(*--itr == std::string("c:"));

      itr_ck = "c:foo/";
      itr = itr_ck.begin();
      BOOST_TEST(*itr == std::string("c:"));
      BOOST_TEST(*++itr == std::string("foo"));
      BOOST_TEST(*++itr == std::string("."));
      BOOST_TEST(++itr == itr_ck.end());
      BOOST_TEST(*--itr == std::string("."));
      BOOST_TEST(*--itr == std::string("foo"));
      BOOST_TEST(*--itr == std::string("c:"));

      itr_ck = path("c:");
      BOOST_TEST(*itr_ck.begin() == std::string("c:"));
      BOOST_TEST(next(itr_ck.begin()) == itr_ck.end());
      BOOST_TEST(prior(itr_ck.end()) == itr_ck.begin());
      BOOST_TEST(*prior(itr_ck.end()) == std::string("c:"));

      itr_ck = path("c:/");
      BOOST_TEST(*itr_ck.begin() == std::string("c:"));
      BOOST_TEST(*next(itr_ck.begin()) == std::string("/"));
      BOOST_TEST(next(next(itr_ck.begin())) == itr_ck.end());
      BOOST_TEST(prior(prior(itr_ck.end())) == itr_ck.begin());
      BOOST_TEST(*prior(itr_ck.end()) == std::string("/"));
      BOOST_TEST(*prior(prior(itr_ck.end())) == std::string("c:"));

      itr_ck = path("c:foo");
      BOOST_TEST(*itr_ck.begin() == std::string("c:"));
      BOOST_TEST(*next(itr_ck.begin()) == std::string("foo"));
      BOOST_TEST(next(next(itr_ck.begin())) == itr_ck.end());
      BOOST_TEST(prior(prior(itr_ck.end())) == itr_ck.begin());
      BOOST_TEST(*prior(itr_ck.end()) == std::string("foo"));
      BOOST_TEST(*prior(prior(itr_ck.end())) == std::string("c:"));

      itr_ck = path("c:/foo");
      BOOST_TEST(*itr_ck.begin() == std::string("c:"));
      BOOST_TEST(*next(itr_ck.begin()) == std::string("/"));
      BOOST_TEST(*next(next(itr_ck.begin())) == std::string("foo"));
      BOOST_TEST(next(next(next(itr_ck.begin()))) == itr_ck.end());
      BOOST_TEST(prior(prior(prior(itr_ck.end()))) == itr_ck.begin());
      BOOST_TEST(*prior(itr_ck.end()) == std::string("foo"));
      BOOST_TEST(*prior(prior(itr_ck.end())) == std::string("/"));
      BOOST_TEST(*prior(prior(prior(itr_ck.end()))) == std::string("c:"));

      itr_ck = path("//net");
      BOOST_TEST(*itr_ck.begin() == std::string("//net"));
      BOOST_TEST(next(itr_ck.begin()) == itr_ck.end());
      BOOST_TEST(prior(itr_ck.end()) == itr_ck.begin());
      BOOST_TEST(*prior(itr_ck.end()) == std::string("//net"));

      itr_ck = path("//net/");
      CHECK_EQUAL(*itr_ck.begin(), "//net");
      CHECK_EQUAL(*next(itr_ck.begin()), "/");
      BOOST_TEST(next(next(itr_ck.begin())) == itr_ck.end());
      BOOST_TEST(prior(prior(itr_ck.end())) == itr_ck.begin());
      CHECK_EQUAL(*prior(itr_ck.end()), "/");
      CHECK_EQUAL(*prior(prior(itr_ck.end())), "//net");

      itr_ck = path("//net/foo");
      BOOST_TEST(*itr_ck.begin() == std::string("//net"));
      BOOST_TEST(*next(itr_ck.begin()) == std::string("/"));
      BOOST_TEST(*next(next(itr_ck.begin())) == std::string("foo"));
      BOOST_TEST(next(next(next(itr_ck.begin()))) == itr_ck.end());
      BOOST_TEST(prior(prior(prior(itr_ck.end()))) == itr_ck.begin());
      BOOST_TEST(*prior(itr_ck.end()) == std::string("foo"));
      BOOST_TEST(*prior(prior(itr_ck.end())) == std::string("/"));
      BOOST_TEST(*prior(prior(prior(itr_ck.end()))) == std::string("//net"));

      itr_ck = path("prn:");
      BOOST_TEST(*itr_ck.begin() == std::string("prn:"));
      BOOST_TEST(next(itr_ck.begin()) == itr_ck.end());
      BOOST_TEST(prior(itr_ck.end()) == itr_ck.begin());
      BOOST_TEST(*prior(itr_ck.end()) == std::string("prn:"));
    }
    else
    {
      itr_ck = "///";
      itr = itr_ck.begin();
      CHECK_EQUAL(*itr,  "/");
      BOOST_TEST(++itr == itr_ck.end());
    }
  }

  //  non_member_tests  ----------------------------------------------------------------//

  void non_member_tests()
  {
    std::cout << "non_member_tests..." << std::endl;

    // test non-member functions, particularly operator overloads
                                                               
    path e, e2;
    std::string es, es2;
    char ecs[] = "";
    char ecs2[] = "";

    char acs[] = "a";
    std::string as(acs);
    path a(as);

    char acs2[] = "a";
    std::string as2(acs2);
    path a2(as2);

    char bcs[] = "b";
    std::string bs(bcs);
    path b(bs);

    // swap
    a.swap(b);
    BOOST_TEST(a.string() == "b");
    BOOST_TEST(b.string() == "a");
    fs::swap(a, b);
    BOOST_TEST(a.string() == "a");
    BOOST_TEST(b.string() == "b");

    // probe operator /
    PATH_CHECK(path("") / ".", ".");
    PATH_CHECK(path("") / "..", "..");
    if (platform == "Windows")
    {
      BOOST_TEST(path("foo\\bar") == "foo/bar");
      BOOST_TEST((b / a).native() == path("b\\a").native());
      BOOST_TEST((bs / a).native() == path("b\\a").native());
      BOOST_TEST((bcs / a).native() == path("b\\a").native());
      BOOST_TEST((b / as).native() == path("b\\a").native());
      BOOST_TEST((b / acs).native() == path("b\\a").native());
      PATH_CHECK(path("a") / "b", "a\\b");
      PATH_CHECK(path("..") / "", "..");
      PATH_CHECK(path("foo") / path("bar"), "foo\\bar"); // path arg
      PATH_CHECK(path("foo") / "bar", "foo\\bar");       // const char* arg
      PATH_CHECK(path("foo") / path("woo/bar").filename(), "foo\\bar"); // const std::string & arg
      PATH_CHECK("foo" / path("bar"), "foo\\bar");
      PATH_CHECK(path("..") / ".." , "..\\..");
      PATH_CHECK(path("/") / ".." , "/..");
      PATH_CHECK(path("/..") / ".." , "/..\\..");
      PATH_CHECK(path("..") / "foo" , "..\\foo");
      PATH_CHECK(path("foo") / ".." , "foo\\..");
      PATH_CHECK(path("..") / "f" , "..\\f");
      PATH_CHECK(path("/..") / "f" , "/..\\f");
      PATH_CHECK(path("f") / ".." , "f\\..");
      PATH_CHECK(path("foo") / ".." / ".." , "foo\\..\\..");
      PATH_CHECK(path("foo") / ".." / ".." / ".." , "foo\\..\\..\\..");
      PATH_CHECK(path("f") / ".." / "b" , "f\\..\\b");
      PATH_CHECK(path("foo") / ".." / "bar" , "foo\\..\\bar");
      PATH_CHECK(path("foo") / "bar" / ".." , "foo\\bar\\..");
      PATH_CHECK(path("foo") / "bar" / ".." / "..", "foo\\bar\\..\\..");
      PATH_CHECK(path("foo") / "bar" / ".." / "blah", "foo\\bar\\..\\blah");
      PATH_CHECK(path("f") / "b" / ".." , "f\\b\\..");
      PATH_CHECK(path("f") / "b" / ".." / "a", "f\\b\\..\\a");
      PATH_CHECK(path("foo") / "bar" / "blah" / ".." / "..", "foo\\bar\\blah\\..\\..");
      PATH_CHECK(path("foo") / "bar" / "blah" / ".." / ".." / "bletch", "foo\\bar\\blah\\..\\..\\bletch");

      PATH_CHECK(path(".") / "foo", ".\\foo");
      PATH_CHECK(path(".") / "..", ".\\..");
      PATH_CHECK(path("foo") / ".", "foo\\.");
      PATH_CHECK(path("..") / ".", "..\\.");
      PATH_CHECK(path(".") / ".", ".\\.");
      PATH_CHECK(path(".") / "." / ".", ".\\.\\.");
      PATH_CHECK(path(".") / "foo" / ".", ".\\foo\\.");
      PATH_CHECK(path("foo") / "." / "bar", "foo\\.\\bar");
      PATH_CHECK(path("foo") / "." / ".", "foo\\.\\.");
      PATH_CHECK(path("foo") / "." / "..", "foo\\.\\..");
      PATH_CHECK(path(".") / "." / "..", ".\\.\\..");
      PATH_CHECK(path(".") / ".." / ".", ".\\..\\.");
      PATH_CHECK(path("..") / "." / ".", "..\\.\\.");
    }
    else  // POSIX
    {
      BOOST_TEST((b / a).string() == "b/a");
      BOOST_TEST((bs / a).string() == "b/a");
      BOOST_TEST((bcs / a).string() == "b/a");
      BOOST_TEST((b / as).string() == "b/a");
      BOOST_TEST((b / acs).string() == "b/a");
      PATH_CHECK(path("a") / "b", "a/b");
      PATH_CHECK(path("..") / "", "..");
      PATH_CHECK(path("") / "..", "..");
      PATH_CHECK(path("foo") / path("bar"), "foo/bar"); // path arg
      PATH_CHECK(path("foo") / "bar", "foo/bar");       // const char* arg
      PATH_CHECK(path("foo") / path("woo/bar").filename(), "foo/bar"); // const std::string & arg
      PATH_CHECK("foo" / path("bar"), "foo/bar");
      PATH_CHECK(path("..") / ".." , "../..");
      PATH_CHECK(path("/") / ".." , "/..");
      PATH_CHECK(path("/..") / ".." , "/../..");
      PATH_CHECK(path("..") / "foo" , "../foo");
      PATH_CHECK(path("foo") / ".." , "foo/..");
      PATH_CHECK(path("..") / "f" , "../f");
      PATH_CHECK(path("/..") / "f" , "/../f");
      PATH_CHECK(path("f") / ".." , "f/..");
      PATH_CHECK(path("foo") / ".." / ".." , "foo/../..");
      PATH_CHECK(path("foo") / ".." / ".." / ".." , "foo/../../..");
      PATH_CHECK(path("f") / ".." / "b" , "f/../b");
      PATH_CHECK(path("foo") / ".." / "bar" , "foo/../bar");
      PATH_CHECK(path("foo") / "bar" / ".." , "foo/bar/..");
      PATH_CHECK(path("foo") / "bar" / ".." / "..", "foo/bar/../..");
      PATH_CHECK(path("foo") / "bar" / ".." / "blah", "foo/bar/../blah");
      PATH_CHECK(path("f") / "b" / ".." , "f/b/..");
      PATH_CHECK(path("f") / "b" / ".." / "a", "f/b/../a");
      PATH_CHECK(path("foo") / "bar" / "blah" / ".." / "..", "foo/bar/blah/../..");
      PATH_CHECK(path("foo") / "bar" / "blah" / ".." / ".." / "bletch", "foo/bar/blah/../../bletch");

      PATH_CHECK(path(".") / "foo", "./foo");
      PATH_CHECK(path(".") / "..", "./..");
      PATH_CHECK(path("foo") / ".", "foo/.");
      PATH_CHECK(path("..") / ".", "../.");
      PATH_CHECK(path(".") / ".", "./.");
      PATH_CHECK(path(".") / "." / ".", "././.");
      PATH_CHECK(path(".") / "foo" / ".", "./foo/.");
      PATH_CHECK(path("foo") / "." / "bar", "foo/./bar");
      PATH_CHECK(path("foo") / "." / ".", "foo/./.");
      PATH_CHECK(path("foo") / "." / "..", "foo/./..");
      PATH_CHECK(path(".") / "." / "..", "././..");
      PATH_CHECK(path(".") / ".." / ".", "./../.");
      PATH_CHECK(path("..") / "." / ".", ".././.");
    }

    // probe operator <
    BOOST_TEST(!(e < e2));
    BOOST_TEST(!(es < e2));
    BOOST_TEST(!(ecs < e2));
    BOOST_TEST(!(e < es2));
    BOOST_TEST(!(e < ecs2));

    BOOST_TEST(e < a);
    BOOST_TEST(es < a);
    BOOST_TEST(ecs < a);
    BOOST_TEST(e < as);
    BOOST_TEST(e < acs);

    BOOST_TEST(a < b);
    BOOST_TEST(as < b);
    BOOST_TEST(acs < b);
    BOOST_TEST(a < bs);
    BOOST_TEST(a < bcs);

    BOOST_TEST(!(a < a2));
    BOOST_TEST(!(as < a2));
    BOOST_TEST(!(acs < a2));
    BOOST_TEST(!(a < as2));
    BOOST_TEST(!(a < acs2));

    // make sure basic_path overloads don't conflict with std::string overloads

    BOOST_TEST(!(as < as));
    BOOST_TEST(!(as < acs));
    BOOST_TEST(!(acs < as));

    // reality check character set is as expected
    BOOST_TEST(std::string("a.b") < std::string("a/b"));
    // verify compare is actually lexicographical
    BOOST_TEST(path("a/b") < path("a.b"));

    // make sure the derivative operators also work

    BOOST_TEST(b > a);
    BOOST_TEST(b > as);
    BOOST_TEST(b > acs);
    BOOST_TEST(bs > a);
    BOOST_TEST(bcs > a);

    BOOST_TEST(!(a2 > a));
    BOOST_TEST(!(a2 > as));
    BOOST_TEST(!(a2 > acs));
    BOOST_TEST(!(as2 > a));
    BOOST_TEST(!(acs2 > a));

    BOOST_TEST(a <= b);
    BOOST_TEST(as <= b);
    BOOST_TEST(acs <= b);
    BOOST_TEST(a <= bs);
    BOOST_TEST(a <= bcs);

    BOOST_TEST(a <= a2);
    BOOST_TEST(as <= a2);
    BOOST_TEST(acs <= a2);
    BOOST_TEST(a <= as2);
    BOOST_TEST(a <= acs2);

    BOOST_TEST(b >= a);
    BOOST_TEST(bs >= a);
    BOOST_TEST(bcs >= a);
    BOOST_TEST(b >= as);
    BOOST_TEST(b >= acs);

    BOOST_TEST(a2 >= a);
    BOOST_TEST(as2 >= a);
    BOOST_TEST(acs2 >= a);
    BOOST_TEST(a2 >= as);
    BOOST_TEST(a2 >= acs);

    //  operator == and != are implemented separately, so test separately

    path p1("fe/fi/fo/fum");
    path p2(p1);
    path p3("fe/fi/fo/fumm");
    BOOST_TEST(p1.string() != p3.string());

    // check each overload
    BOOST_TEST(p1 != p3);
    BOOST_TEST(p1 != p3.string());
    BOOST_TEST(p1 != p3.string().c_str());
    BOOST_TEST(p1.string() != p3);
    BOOST_TEST(p1.string().c_str() != p3);

    p3 = p2;
    BOOST_TEST(p1.string() == p3.string());

    // check each overload
    BOOST_TEST(p1 == p3);
    BOOST_TEST(p1 == p3.string());
    BOOST_TEST(p1 == p3.string().c_str());
    BOOST_TEST(p1.string() == p3);
    BOOST_TEST(p1.string().c_str() == p3);

    if (platform == "Windows")
    {
      std::cout << "Windows relational tests..." << std::endl;
      path p10 ("c:\\file");
      path p11 ("c:/file");
      // check each overload
      BOOST_TEST(p10.generic_string() == p11.generic_string());
      BOOST_TEST(p10 == p11);
      BOOST_TEST(p10 == p11.string());
      BOOST_TEST(p10 == p11.string().c_str());
      BOOST_TEST(p10.string() == p11);
      BOOST_TEST(p10.string().c_str() == p11);
      BOOST_TEST(p10 == L"c:\\file");
      BOOST_TEST(p10 == L"c:/file");
      BOOST_TEST(p11 == L"c:\\file");
      BOOST_TEST(p11 == L"c:/file");
      BOOST_TEST(L"c:\\file" == p10);
      BOOST_TEST(L"c:/file" == p10);
      BOOST_TEST(L"c:\\file" == p11);
      BOOST_TEST(L"c:/file" == p11);

      BOOST_TEST(!(p10.generic_string() != p11.generic_string()));
      BOOST_TEST(!(p10 != p11));
      BOOST_TEST(!(p10 != p11.string()));
      BOOST_TEST(!(p10 != p11.string().c_str()));
      BOOST_TEST(!(p10.string() != p11));
      BOOST_TEST(!(p10.string().c_str() != p11));
      BOOST_TEST(!(p10 != L"c:\\file"));
      BOOST_TEST(!(p10 != L"c:/file"));
      BOOST_TEST(!(p11 != L"c:\\file"));
      BOOST_TEST(!(p11 != L"c:/file"));
      BOOST_TEST(!(L"c:\\file" != p10));
      BOOST_TEST(!(L"c:/file" != p10));
      BOOST_TEST(!(L"c:\\file" != p11));
      BOOST_TEST(!(L"c:/file" != p11));

      BOOST_TEST(!(p10.string() < p11.string()));
      BOOST_TEST(!(p10 < p11));
      BOOST_TEST(!(p10 < p11.string()));
      BOOST_TEST(!(p10 < p11.string().c_str()));
      BOOST_TEST(!(p10.string() < p11));
      BOOST_TEST(!(p10.string().c_str() < p11));
      BOOST_TEST(!(p10 < L"c:\\file"));
      BOOST_TEST(!(p10 < L"c:/file"));
      BOOST_TEST(!(p11 < L"c:\\file"));
      BOOST_TEST(!(p11 < L"c:/file"));
      BOOST_TEST(!(L"c:\\file" < p10));
      BOOST_TEST(!(L"c:/file" < p10));
      BOOST_TEST(!(L"c:\\file" < p11));
      BOOST_TEST(!(L"c:/file" < p11));

      BOOST_TEST(!(p10.generic_string() > p11.generic_string()));
      BOOST_TEST(!(p10 > p11));
      BOOST_TEST(!(p10 > p11.string()));
      BOOST_TEST(!(p10 > p11.string().c_str()));
      BOOST_TEST(!(p10.string() > p11));
      BOOST_TEST(!(p10.string().c_str() > p11));
      BOOST_TEST(!(p10 > L"c:\\file"));
      BOOST_TEST(!(p10 > L"c:/file"));
      BOOST_TEST(!(p11 > L"c:\\file"));
      BOOST_TEST(!(p11 > L"c:/file"));
      BOOST_TEST(!(L"c:\\file" > p10));
      BOOST_TEST(!(L"c:/file" > p10));
      BOOST_TEST(!(L"c:\\file" > p11));
      BOOST_TEST(!(L"c:/file" > p11));
    }
  }

  //  query_and_decomposition_tests  ---------------------------------------------------//
  //
  //  remove_filename() is also tested here, because its specification depends on
  //  a decomposition function.

  void query_and_decomposition_tests()
  {
    std::cout << "query_and_decomposition_tests..." << std::endl;

    // stem() tests not otherwise covered
    BOOST_TEST(path("b").stem() == "b");
    BOOST_TEST(path("a/b.txt").stem() == "b");
    BOOST_TEST(path("a/b.").stem() == "b"); 
    BOOST_TEST(path("a.b.c").stem() == "a.b");
    BOOST_TEST(path("a.b.c.").stem() == "a.b.c");

    // extension() tests not otherwise covered
    BOOST_TEST(path("a/b").extension() == "");
    BOOST_TEST(path("a.b/c").extension() == "");
    BOOST_TEST(path("a/b.txt").extension() == ".txt");
    BOOST_TEST(path("a/b.").extension() == ".");
    BOOST_TEST(path("a.b.c").extension() == ".c");
    BOOST_TEST(path("a.b.c.").extension() == ".");
    BOOST_TEST(path("a/").extension() == "");

    // main q & d test sequence
    path p;
    path q;

    p = q = "";
    BOOST_TEST(p.relative_path().string() == "");
    BOOST_TEST(p.parent_path().string() == "");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    BOOST_TEST(p.filename() == "");
    BOOST_TEST(p.stem() == "");
    BOOST_TEST(p.extension() == "");
    BOOST_TEST(p.root_name() == "");
    BOOST_TEST(p.root_directory() == "");
    BOOST_TEST(p.root_path().string() == "");
    BOOST_TEST(!p.has_root_path());
    BOOST_TEST(!p.has_root_name());
    BOOST_TEST(!p.has_root_directory());
    BOOST_TEST(!p.has_relative_path());
    BOOST_TEST(!p.has_filename());
    BOOST_TEST(!p.has_stem());
    BOOST_TEST(!p.has_extension());
    BOOST_TEST(!p.has_parent_path());
    BOOST_TEST(!p.is_absolute());

    p = q = "/";
    BOOST_TEST(p.relative_path().string() == "");
    BOOST_TEST(p.parent_path().string() == "");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    BOOST_TEST(p.filename() == "/");
    BOOST_TEST(p.stem() == "/");
    BOOST_TEST(p.extension() == "");
    BOOST_TEST(p.root_name() == "");
    BOOST_TEST(p.root_directory() == "/");
    BOOST_TEST(p.root_path().string() == "/");
    BOOST_TEST(p.has_root_path());
    BOOST_TEST(!p.has_root_name());
    BOOST_TEST(p.has_root_directory());
    BOOST_TEST(!p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_stem());
    BOOST_TEST(!p.has_extension());
    BOOST_TEST(!p.has_parent_path());
    if (platform == "POSIX")
      BOOST_TEST(p.is_absolute());
    else
      BOOST_TEST(!p.is_absolute());

    p = q = "//";
    CHECK_EQUAL(p.relative_path().string(), "");
    CHECK_EQUAL(p.parent_path().string(), "");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    CHECK_EQUAL(p.filename(), "//");
    CHECK_EQUAL(p.stem(), "//");
    CHECK_EQUAL(p.extension(), "");
    CHECK_EQUAL(p.root_name(), "//");
    CHECK_EQUAL(p.root_directory(), "");
    CHECK_EQUAL(p.root_path().string(), "//");
    BOOST_TEST(p.has_root_path());
    BOOST_TEST(p.has_root_name());
    BOOST_TEST(!p.has_root_directory());
    BOOST_TEST(!p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_stem());
    BOOST_TEST(!p.has_extension());
    BOOST_TEST(!p.has_parent_path());
    BOOST_TEST(!p.is_absolute());

    p = q = "///";
    CHECK_EQUAL(p.relative_path().string(), "");
    CHECK_EQUAL(p.parent_path().string(), "");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    CHECK_EQUAL(p.filename(), "/");
    CHECK_EQUAL(p.stem(), "/");
    CHECK_EQUAL(p.extension(), "");
    CHECK_EQUAL(p.root_name(), "");
    CHECK_EQUAL(p.root_directory(), "/");
    CHECK_EQUAL(p.root_path().string(), "/");
    BOOST_TEST(p.has_root_path());
    BOOST_TEST(!p.has_root_name());
    BOOST_TEST(p.has_root_directory());
    BOOST_TEST(!p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_stem());
    BOOST_TEST(!p.has_extension());
    BOOST_TEST(!p.has_parent_path());
    if (platform == "POSIX")
      BOOST_TEST(p.is_absolute());
    else
      BOOST_TEST(!p.is_absolute());

    p = q = ".";
    BOOST_TEST(p.relative_path().string() == ".");
    BOOST_TEST(p.parent_path().string() == "");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    BOOST_TEST(p.filename() == ".");
    BOOST_TEST(p.stem() == ".");
    BOOST_TEST(p.extension() == "");
    BOOST_TEST(p.root_name() == "");
    BOOST_TEST(p.root_directory() == "");
    BOOST_TEST(p.root_path().string() == "");
    BOOST_TEST(!p.has_root_path());
    BOOST_TEST(!p.has_root_name());
    BOOST_TEST(!p.has_root_directory());
    BOOST_TEST(p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_stem());
    BOOST_TEST(!p.has_extension());
    BOOST_TEST(!p.has_parent_path());
    BOOST_TEST(!p.is_absolute());

    p = q = "..";
    BOOST_TEST(p.relative_path().string() == "..");
    BOOST_TEST(p.parent_path().string() == "");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    BOOST_TEST(p.filename() == "..");
    BOOST_TEST(p.stem() == "..");
    BOOST_TEST(p.extension() == "");
    BOOST_TEST(p.root_name() == "");
    BOOST_TEST(p.root_directory() == "");
    BOOST_TEST(p.root_path().string() == "");
    BOOST_TEST(!p.has_root_path());
    BOOST_TEST(!p.has_root_name());
    BOOST_TEST(!p.has_root_directory());
    BOOST_TEST(p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_stem());
    BOOST_TEST(!p.has_extension());
    BOOST_TEST(!p.has_parent_path());
    BOOST_TEST(!p.is_absolute());

    p = q = "foo";
    BOOST_TEST(p.relative_path().string() == "foo");
    BOOST_TEST(p.parent_path().string() == "");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    BOOST_TEST(p.filename() == "foo");
    BOOST_TEST(p.stem() == "foo");
    BOOST_TEST(p.extension() == "");
    BOOST_TEST(p.root_name() == "");
    BOOST_TEST(p.root_directory() == "");
    BOOST_TEST(p.root_path().string() == "");
    BOOST_TEST(!p.has_root_path());
    BOOST_TEST(!p.has_root_name());
    BOOST_TEST(!p.has_root_directory());
    BOOST_TEST(p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_stem());
    BOOST_TEST(!p.has_extension());
    BOOST_TEST(!p.has_parent_path());
    BOOST_TEST(!p.is_absolute());

    p = q = "/foo";
    CHECK_EQUAL(p.relative_path().string(), "foo");
    CHECK_EQUAL(p.parent_path().string(), "/");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    CHECK_EQUAL(p.filename(), "foo");
    CHECK_EQUAL(p.stem(), "foo");
    CHECK_EQUAL(p.extension(), "");
    CHECK_EQUAL(p.root_name(), "");
    CHECK_EQUAL(p.root_directory(), "/");
    CHECK_EQUAL(p.root_path().string(), "/");
    BOOST_TEST(p.has_root_path());
    BOOST_TEST(!p.has_root_name());
    BOOST_TEST(p.has_root_directory());
    BOOST_TEST(p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_stem());
    BOOST_TEST(!p.has_extension());
    BOOST_TEST(p.has_parent_path());
    if (platform == "POSIX")
      BOOST_TEST(p.is_absolute());
    else
      BOOST_TEST(!p.is_absolute());

    p = q = "/foo/";
    CHECK_EQUAL(p.relative_path().string(), "foo/");
    CHECK_EQUAL(p.parent_path().string(), "/foo");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    CHECK_EQUAL(p.filename(), ".");
    CHECK_EQUAL(p.stem(), ".");
    CHECK_EQUAL(p.extension(), "");
    CHECK_EQUAL(p.root_name(), "");
    CHECK_EQUAL(p.root_directory(), "/");
    CHECK_EQUAL(p.root_path().string(), "/");
    BOOST_TEST(p.has_root_path());
    BOOST_TEST(!p.has_root_name());
    BOOST_TEST(p.has_root_directory());
    BOOST_TEST(p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_stem());
    BOOST_TEST(!p.has_extension());
    BOOST_TEST(p.has_parent_path());
    if (platform == "POSIX")
      BOOST_TEST(p.is_absolute());
    else
      BOOST_TEST(!p.is_absolute());

    p = q = "///foo";
    CHECK_EQUAL(p.relative_path().string(), "foo");
    CHECK_EQUAL(p.parent_path().string(), "/");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    CHECK_EQUAL(p.filename(), "foo");
    CHECK_EQUAL(p.root_name(), "");
    CHECK_EQUAL(p.root_directory(), "/");
    CHECK_EQUAL(p.root_path().string(), "/");
    BOOST_TEST(p.has_root_path());
    BOOST_TEST(!p.has_root_name());
    BOOST_TEST(p.has_root_directory());
    BOOST_TEST(p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_parent_path());
    if (platform == "POSIX")
      BOOST_TEST(p.is_absolute());
    else
      BOOST_TEST(!p.is_absolute());

    p = q = "foo/bar";
    BOOST_TEST(p.relative_path().string() == "foo/bar");
    BOOST_TEST(p.parent_path().string() == "foo");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    BOOST_TEST(p.filename() == "bar");
    BOOST_TEST(p.stem() == "bar");
    BOOST_TEST(p.extension() == "");
    BOOST_TEST(p.root_name() == "");
    BOOST_TEST(p.root_directory() == "");
    BOOST_TEST(p.root_path().string() == "");
    BOOST_TEST(!p.has_root_path());
    BOOST_TEST(!p.has_root_name());
    BOOST_TEST(!p.has_root_directory());
    BOOST_TEST(p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_stem());
    BOOST_TEST(!p.has_extension());
    BOOST_TEST(p.has_parent_path());
    BOOST_TEST(!p.is_absolute());

    p = q = "../foo";
    BOOST_TEST(p.relative_path().string() == "../foo");
    BOOST_TEST(p.parent_path().string() == "..");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    BOOST_TEST(p.filename() == "foo");
    BOOST_TEST(p.root_name() == "");
    BOOST_TEST(p.root_directory() == "");
    BOOST_TEST(p.root_path().string() == "");
    BOOST_TEST(!p.has_root_path());
    BOOST_TEST(!p.has_root_name());
    BOOST_TEST(!p.has_root_directory());
    BOOST_TEST(p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_parent_path());
    BOOST_TEST(!p.is_absolute());

    p = q = "..///foo";
    CHECK_EQUAL(p.relative_path().string(), "..///foo");
    CHECK_EQUAL(p.parent_path().string(), "..");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    CHECK_EQUAL(p.filename(), "foo");
    CHECK_EQUAL(p.root_name(), "");
    CHECK_EQUAL(p.root_directory(), "");
    CHECK_EQUAL(p.root_path().string(), "");
    BOOST_TEST(!p.has_root_path());
    BOOST_TEST(!p.has_root_name());
    BOOST_TEST(!p.has_root_directory());
    BOOST_TEST(p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_parent_path());
    BOOST_TEST(!p.is_absolute());

    p = q = "/foo/bar";
    BOOST_TEST(p.relative_path().string() == "foo/bar");
    BOOST_TEST(p.parent_path().string() == "/foo");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    BOOST_TEST(p.filename() == "bar");
    BOOST_TEST(p.root_name() == "");
    BOOST_TEST(p.root_directory() == "/");
    BOOST_TEST(p.root_path().string() == "/");
    BOOST_TEST(p.has_root_path());
    BOOST_TEST(!p.has_root_name());
    BOOST_TEST(p.has_root_directory());
    BOOST_TEST(p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_parent_path());
    if (platform == "POSIX")
      BOOST_TEST(p.is_absolute());
    else
      BOOST_TEST(!p.is_absolute());

    // Both POSIX and Windows allow two leading slashs
    // (POSIX meaning is implementation defined)
    PATH_CHECK(path("//resource"), "//resource");
    PATH_CHECK(path("//resource/"), "//resource/");
    PATH_CHECK(path("//resource/foo"), "//resource/foo");

    p = q = path("//net");
    CHECK_EQUAL(p.string(), "//net");
    CHECK_EQUAL(p.relative_path().string(), "");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    CHECK_EQUAL(p.parent_path().string(), "");
    CHECK_EQUAL(p.filename(), "//net");
    CHECK_EQUAL(p.root_name(), "//net");
    CHECK_EQUAL(p.root_directory(), "");
    CHECK_EQUAL(p.root_path().string(), "//net");
    BOOST_TEST(p.has_root_path());
    BOOST_TEST(p.has_root_name());
    BOOST_TEST(!p.has_root_directory());
    BOOST_TEST(!p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(!p.has_parent_path());
    BOOST_TEST(!p.is_absolute());

    p = q = path("//net/");
    BOOST_TEST(p.relative_path().string() == "");
    BOOST_TEST(p.parent_path().string() == "//net");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    BOOST_TEST(p.filename() == "/");
    BOOST_TEST(p.root_name() == "//net");
    BOOST_TEST(p.root_directory() == "/");
    BOOST_TEST(p.root_path().string() == "//net/");
    BOOST_TEST(p.has_root_path());
    BOOST_TEST(p.has_root_name());
    BOOST_TEST(p.has_root_directory());
    BOOST_TEST(!p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_parent_path());
    BOOST_TEST(p.is_absolute());

    p = q = path("//net/foo");
    BOOST_TEST(p.relative_path().string() == "foo");
    BOOST_TEST(p.parent_path().string() == "//net/");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    BOOST_TEST(p.filename() == "foo");
    BOOST_TEST(p.root_name() == "//net");
    BOOST_TEST(p.root_directory() == "/");
    BOOST_TEST(p.root_path().string() == "//net/");
    BOOST_TEST(p.has_root_path());
    BOOST_TEST(p.has_root_name());
    BOOST_TEST(p.has_root_directory());
    BOOST_TEST(p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_parent_path());
    BOOST_TEST(p.is_absolute());

    p = q = path("//net///foo");
    CHECK_EQUAL(p.relative_path().string(), "foo");
    CHECK_EQUAL(p.parent_path().string(), "//net/");
    BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
    CHECK_EQUAL(p.filename(), "foo");
    CHECK_EQUAL(p.root_name(), "//net");
    CHECK_EQUAL(p.root_directory(), "/");
    CHECK_EQUAL(p.root_path().string(), "//net/");
    BOOST_TEST(p.has_root_path());
    BOOST_TEST(p.has_root_name());
    BOOST_TEST(p.has_root_directory());
    BOOST_TEST(p.has_relative_path());
    BOOST_TEST(p.has_filename());
    BOOST_TEST(p.has_parent_path());
    BOOST_TEST(p.is_absolute());

    if (platform == "Windows")
    {
 
      //p = q = L"\\\\?\\";
      //BOOST_TEST(p.relative_path().string() == "");
      //BOOST_TEST(p.parent_path().string() == "");
      //BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
      //BOOST_TEST(p.filename() == "");
      //BOOST_TEST(p.stem() == "");
      //BOOST_TEST(p.extension() == "");
      //BOOST_TEST(p.root_name() == "");
      //BOOST_TEST(p.root_directory() == "");
      //BOOST_TEST(p.root_path().string() == "");
      //BOOST_TEST(!p.has_root_path());
      //BOOST_TEST(!p.has_root_name());
      //BOOST_TEST(!p.has_root_directory());
      //BOOST_TEST(!p.has_relative_path());
      //BOOST_TEST(!p.has_filename());
      //BOOST_TEST(!p.has_stem());
      //BOOST_TEST(!p.has_extension());
      //BOOST_TEST(!p.has_parent_path());
      //BOOST_TEST(!p.is_absolute());

      p = q = path("c:");
      BOOST_TEST(p.relative_path().string() == "");
      BOOST_TEST(p.parent_path().string() == "");
      BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
      BOOST_TEST(p.filename() == "c:");
      BOOST_TEST(p.root_name() == "c:");
      BOOST_TEST(p.root_directory() == "");
      BOOST_TEST(p.root_path().string() == "c:");
      BOOST_TEST(p.has_root_path());
      BOOST_TEST(p.has_root_name());
      BOOST_TEST(!p.has_root_directory());
      BOOST_TEST(!p.has_relative_path());
      BOOST_TEST(p.has_filename());
      BOOST_TEST(!p.has_parent_path());
      BOOST_TEST(!p.is_absolute());
 
      //p = q = path(L"\\\\?\\c:");
      //BOOST_TEST(p.relative_path().string() == "");
      //BOOST_TEST(p.parent_path().string() == "");
      //BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
      //BOOST_TEST(p.filename() == "c:");
      //BOOST_TEST(p.root_name() == "c:");
      //BOOST_TEST(p.root_directory() == "");
      //BOOST_TEST(p.root_path().string() == "c:");
      //BOOST_TEST(p.has_root_path());
      //BOOST_TEST(p.has_root_name());
      //BOOST_TEST(!p.has_root_directory());
      //BOOST_TEST(!p.has_relative_path());
      //BOOST_TEST(p.has_filename());
      //BOOST_TEST(!p.has_parent_path());
      //BOOST_TEST(!p.is_absolute());

      p = q = path("c:foo");
      BOOST_TEST(p.relative_path().string() == "foo");
      BOOST_TEST(p.parent_path().string() == "c:");
      BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
      BOOST_TEST(p.filename() == "foo");
      BOOST_TEST(p.root_name() == "c:");
      BOOST_TEST(p.root_directory() == "");
      BOOST_TEST(p.root_path().string() == "c:");
      BOOST_TEST(p.has_root_path());
      BOOST_TEST(p.has_root_name());
      BOOST_TEST(!p.has_root_directory());
      BOOST_TEST(p.has_relative_path());
      BOOST_TEST(p.has_filename());
      BOOST_TEST(p.has_parent_path());
      BOOST_TEST(!p.is_absolute());

      //p = q = path(L"\\\\?\\c:foo");
      //BOOST_TEST(p.relative_path().string() == "foo");
      //BOOST_TEST(p.parent_path().string() == "c:");
      //BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
      //BOOST_TEST(p.filename() == "foo");
      //BOOST_TEST(p.root_name() == "c:");
      //BOOST_TEST(p.root_directory() == "");
      //BOOST_TEST(p.root_path().string() == "c:");
      //BOOST_TEST(p.has_root_path());
      //BOOST_TEST(p.has_root_name());
      //BOOST_TEST(!p.has_root_directory());
      //BOOST_TEST(p.has_relative_path());
      //BOOST_TEST(p.has_filename());
      //BOOST_TEST(p.has_parent_path());
      //BOOST_TEST(!p.is_absolute());
   
      p = q = path("c:/");
      BOOST_TEST(p.relative_path().string() == "");
      BOOST_TEST(p.parent_path().string() == "c:");
      BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
      BOOST_TEST(p.filename() == "/");
      BOOST_TEST(p.root_name() == "c:");
      BOOST_TEST(p.root_directory() == "/");
      BOOST_TEST(p.root_path().string() == "c:/");
      BOOST_TEST(p.has_root_path());
      BOOST_TEST(p.has_root_name());
      BOOST_TEST(p.has_root_directory());
      BOOST_TEST(!p.has_relative_path());
      BOOST_TEST(p.has_filename());
      BOOST_TEST(p.has_parent_path());
      BOOST_TEST(p.is_absolute());

      p = q = path("c:..");
      BOOST_TEST(p.relative_path().string() == "..");
      BOOST_TEST(p.parent_path().string() == "c:");
      BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
      BOOST_TEST(p.filename() == "..");
      BOOST_TEST(p.root_name() == "c:");
      BOOST_TEST(p.root_directory() == "");
      BOOST_TEST(p.root_path().string() == "c:");
      BOOST_TEST(p.has_root_path());
      BOOST_TEST(p.has_root_name());
      BOOST_TEST(!p.has_root_directory());
      BOOST_TEST(p.has_relative_path());
      BOOST_TEST(p.has_filename());
      BOOST_TEST(p.has_parent_path());
      BOOST_TEST(!p.is_absolute());

      p = q = path("c:/foo");
      CHECK_EQUAL(p.relative_path().string(), "foo");
      CHECK_EQUAL(p.parent_path().string(), "c:/");
      BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
      CHECK_EQUAL(p.filename(), "foo");
      CHECK_EQUAL(p.root_name(), "c:");
      CHECK_EQUAL(p.root_directory(), "/");
      CHECK_EQUAL(p.root_path().string(), "c:/");
      BOOST_TEST(p.has_root_path());
      BOOST_TEST(p.has_root_name());
      BOOST_TEST(p.has_root_directory());
      BOOST_TEST(p.has_relative_path());
      BOOST_TEST(p.has_filename());
      BOOST_TEST(p.has_parent_path());
      BOOST_TEST(p.is_absolute());

      p = q = path("c://foo");
      CHECK_EQUAL(p.relative_path().string(), "foo");
      CHECK_EQUAL(p.parent_path().string(), "c:/");
      BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
      CHECK_EQUAL(p.filename(), "foo");
      CHECK_EQUAL(p.root_name(), "c:");
      CHECK_EQUAL(p.root_directory(), "/");
      CHECK_EQUAL(p.root_path().string(), "c:/");
      BOOST_TEST(p.has_root_path());
      BOOST_TEST(p.has_root_name());
      BOOST_TEST(p.has_root_directory());
      BOOST_TEST(p.has_relative_path());
      BOOST_TEST(p.has_filename());
      BOOST_TEST(p.has_parent_path());
      BOOST_TEST(p.is_absolute());

      p = q = path("c:\\foo\\bar");
      CHECK_EQUAL(p.relative_path().string(), "foo\\bar");
      CHECK_EQUAL(p.parent_path().string(), "c:\\foo");
      BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
      CHECK_EQUAL(p.filename(), "bar");
      CHECK_EQUAL(p.root_name(), "c:");
      CHECK_EQUAL(p.root_directory(), "\\");
      CHECK_EQUAL(p.root_path().string(), "c:\\");
      BOOST_TEST(p.has_root_path());
      BOOST_TEST(p.has_root_name());
      BOOST_TEST(p.has_root_directory());
      BOOST_TEST(p.has_relative_path());
      BOOST_TEST(p.has_filename());
      BOOST_TEST(p.has_parent_path());
      BOOST_TEST(p.is_absolute());

      p = q = path("prn:");
      BOOST_TEST(p.relative_path().string() == "");
      BOOST_TEST(p.parent_path().string() == "");
      BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
      BOOST_TEST(p.filename() == "prn:");
      BOOST_TEST(p.root_name() == "prn:");
      BOOST_TEST(p.root_directory() == "");
      BOOST_TEST(p.root_path().string() == "prn:");
      BOOST_TEST(p.has_root_path());
      BOOST_TEST(p.has_root_name());
      BOOST_TEST(!p.has_root_directory());
      BOOST_TEST(!p.has_relative_path());
      BOOST_TEST(p.has_filename());
      BOOST_TEST(!p.has_parent_path());
      BOOST_TEST(!p.is_absolute());

      p = q = path("\\\\net\\\\\\foo");
      CHECK_EQUAL(p.relative_path().string(), "foo");
      CHECK_EQUAL(p.parent_path().string(), "\\\\net\\");
      BOOST_TEST_EQ(q.remove_filename().string(), p.parent_path().string());
      CHECK_EQUAL(p.filename(), "foo");
      CHECK_EQUAL(p.root_name(), "\\\\net");
      CHECK_EQUAL(p.root_directory(), "\\");
      CHECK_EQUAL(p.root_path().string(), "\\\\net\\");
      BOOST_TEST(p.has_root_path());
      BOOST_TEST(p.has_root_name());
      BOOST_TEST(p.has_root_directory());
      BOOST_TEST(p.has_relative_path());
      BOOST_TEST(p.has_filename());
      BOOST_TEST(p.has_parent_path());
      BOOST_TEST(p.is_absolute());
    } // Windows

    else
    { // POSIX
      PATH_CHECK(path("/foo/bar/"), "/foo/bar/");
      PATH_CHECK(path("//foo//bar//"), "//foo//bar//");
      PATH_CHECK(path("///foo///bar///"), "///foo///bar///");

      p = path("/usr/local/bin:/usr/bin:/bin");
      BOOST_TEST(p.string() == "/usr/local/bin:/usr/bin:/bin");
    } // POSIX
  }

 //  composition_tests  ----------------------------------------------------------------//

  void composition_tests()
  {
    std::cout << "composition_tests..." << std::endl;

  }

 //  construction_tests  ---------------------------------------------------------------//

  void construction_tests()
  {
    std::cout << "construction_tests..." << std::endl;

    PATH_CHECK("", "");

    PATH_CHECK("foo", "foo");
    PATH_CHECK("f", "f");

    PATH_CHECK("foo/", "foo/");
    PATH_CHECK("f/", "f/");
    PATH_CHECK("foo/..", "foo/..");
    PATH_CHECK("foo/../", "foo/../");
    PATH_CHECK("foo/bar/../..", "foo/bar/../..");
    PATH_CHECK("foo/bar/../../", "foo/bar/../../");
    PATH_CHECK(path("/"), "/");
    PATH_CHECK(path("/f"), "/f");

    PATH_CHECK("/foo", "/foo");
    PATH_CHECK(path("/foo/bar/"), "/foo/bar/");
    PATH_CHECK(path("//foo//bar//"), "//foo//bar//");
    PATH_CHECK(path("///foo///bar///"), "///foo///bar///");
    PATH_CHECK(path("\\/foo\\/bar\\/"), "\\/foo\\/bar\\/");
    PATH_CHECK(path("\\//foo\\//bar\\//"), "\\//foo\\//bar\\//");

    if (platform == "Windows")
    {
      PATH_CHECK(path("c:") / "foo", "c:foo");
      PATH_CHECK(path("c:") / "/foo", "c:/foo");

      PATH_CHECK(path("\\foo\\bar\\"), "\\foo\\bar\\");
      PATH_CHECK(path("\\\\foo\\\\bar\\\\"), "\\\\foo\\\\bar\\\\");
      PATH_CHECK(path("\\\\\\foo\\\\\\bar\\\\\\"), "\\\\\\foo\\\\\\bar\\\\\\");

      PATH_CHECK(path("\\"), "\\");
      PATH_CHECK(path("\\f"), "\\f");
      PATH_CHECK(path("\\foo"), "\\foo");
      PATH_CHECK(path("foo\\bar"), "foo\\bar");
      PATH_CHECK(path("foo bar"), "foo bar");
      PATH_CHECK(path("c:"), "c:");
      PATH_CHECK(path("c:/"), "c:/");
      PATH_CHECK(path("c:."), "c:.");
      PATH_CHECK(path("c:./foo"), "c:./foo");
      PATH_CHECK(path("c:.\\foo"), "c:.\\foo");
      PATH_CHECK(path("c:.."), "c:..");
      PATH_CHECK(path("c:/."), "c:/.");
      PATH_CHECK(path("c:/.."), "c:/..");
      PATH_CHECK(path("c:/../"), "c:/../");
      PATH_CHECK(path("c:\\..\\"), "c:\\..\\");
      PATH_CHECK(path("c:/../.."), "c:/../..");
      PATH_CHECK(path("c:/../foo"), "c:/../foo");
      PATH_CHECK(path("c:\\..\\foo"), "c:\\..\\foo");
      PATH_CHECK(path("c:../foo"), "c:../foo");
      PATH_CHECK(path("c:..\\foo"), "c:..\\foo");
      PATH_CHECK(path("c:/../../foo"), "c:/../../foo");
      PATH_CHECK(path("c:\\..\\..\\foo"), "c:\\..\\..\\foo");
      PATH_CHECK(path("c:foo/.."), "c:foo/..");
      PATH_CHECK(path("c:/foo/.."), "c:/foo/..");
      PATH_CHECK(path("c:/..foo"), "c:/..foo");
      PATH_CHECK(path("c:foo"), "c:foo");
      PATH_CHECK(path("c:/foo"), "c:/foo");
      PATH_CHECK(path("\\\\netname"), "\\\\netname");
      PATH_CHECK(path("\\\\netname\\"), "\\\\netname\\");
      PATH_CHECK(path("\\\\netname\\foo"), "\\\\netname\\foo");
      PATH_CHECK(path("c:/foo"), "c:/foo");
      PATH_CHECK(path("prn:"), "prn:");
    }
    else
    {
    }

    PATH_CHECK("foo/bar", "foo/bar");
    PATH_CHECK("a/b", "a/b");  // probe for length effects
    PATH_CHECK("..", "..");
    PATH_CHECK("../..", "../..");
    PATH_CHECK("/..", "/..");
    PATH_CHECK("/../..", "/../..");
    PATH_CHECK("../foo", "../foo");
    PATH_CHECK("foo/..", "foo/..");
    PATH_CHECK(path("foo/..bar"), "foo/..bar");
    PATH_CHECK("../f", "../f");
    PATH_CHECK("/../f", "/../f");
    PATH_CHECK("f/..", "f/..");
    PATH_CHECK("foo/../..", "foo/../..");
    PATH_CHECK("foo/../../..", "foo/../../..");
    PATH_CHECK("foo/../bar", "foo/../bar");
    PATH_CHECK("foo/bar/..", "foo/bar/..");
    PATH_CHECK("foo/bar/../..", "foo/bar/../..");
    PATH_CHECK("foo/bar/../blah", "foo/bar/../blah");
    PATH_CHECK("f/../b", "f/../b");
    PATH_CHECK("f/b/..", "f/b/..");
    PATH_CHECK("f/b/../a", "f/b/../a");
    PATH_CHECK("foo/bar/blah/../..", "foo/bar/blah/../..");
    PATH_CHECK("foo/bar/blah/../../bletch", "foo/bar/blah/../../bletch");
    PATH_CHECK("...", "...");
    PATH_CHECK("....", "....");
    PATH_CHECK("foo/...", "foo/...");
    PATH_CHECK("abc.", "abc.");
    PATH_CHECK("abc..", "abc..");
    PATH_CHECK("foo/abc.", "foo/abc.");
    PATH_CHECK("foo/abc..", "foo/abc..");

    PATH_CHECK(path(".abc"), ".abc");
    PATH_CHECK("a.c", "a.c");
    PATH_CHECK(path("..abc"), "..abc");
    PATH_CHECK("a..c", "a..c");
    PATH_CHECK(path("foo/.abc"), "foo/.abc");
    PATH_CHECK("foo/a.c", "foo/a.c");
    PATH_CHECK(path("foo/..abc"), "foo/..abc");
    PATH_CHECK("foo/a..c", "foo/a..c");

    PATH_CHECK(".", ".");
    PATH_CHECK("./foo", "./foo");
    PATH_CHECK("./..", "./..");
    PATH_CHECK("./../foo", "./../foo");
    PATH_CHECK("foo/.", "foo/.");
    PATH_CHECK("../.", "../.");
    PATH_CHECK("./.", "./.");
    PATH_CHECK("././.", "././.");
    PATH_CHECK("./foo/.", "./foo/.");
    PATH_CHECK("foo/./bar", "foo/./bar");
    PATH_CHECK("foo/./.", "foo/./.");
    PATH_CHECK("foo/./..", "foo/./..");
    PATH_CHECK("foo/./../bar", "foo/./../bar");
    PATH_CHECK("foo/../.", "foo/../.");
    PATH_CHECK("././..", "././..");
    PATH_CHECK("./../.", "./../.");
    PATH_CHECK(".././.", ".././.");
  }

  //  append_tests  --------------------------------------------------------------------//

  void append_test_aux(const path & p, const std::string & s, const std::string & expect)
  {
    PATH_CHECK(p / path(s), expect);
    PATH_CHECK(p / s.c_str(), expect);
    PATH_CHECK(p / s, expect);
    path x(p);
    x.append(s.begin(), s.end());
    PATH_CHECK(x, expect);
  }

  void append_tests()
  {
    std::cout << "append_tests..." << std::endl;

    // There are many control paths to be exercised, since empty paths and arguments,
    // paths with trailing separators, arguments with leading separators, with or without
    // other characters being present, are all separate cases that need to be tested.
    // Furthermore, some of the code to be tested is specific to argument categories,
    // so that results in further permutations to be tested.

    //// code to generate test cases
    //// 
    //// expected results must be checked by hand
    //// "foo\bar" expected result must be edited by hand and moved for Windows/POSIX
    ////
    //const char* x[]    = { "", "/", "foo", "foo/" };
    //const char* y[] = { "", "/", "bar", "/bar" };

    //for (int i = 0; i < sizeof(x)/sizeof(char*); ++i)
    //  for (int j = 0; j < sizeof(y)/sizeof(char*); ++j)
    //  {
    //    std::cout << "\n    PATH_CHECK(path(\"" << x[i] << "\") / \"" << y[j] << "\", \"" 
    //              << path(x[i]) / y[j] << "\");\n";
    //    std::cout << "    append_test_aux(\"" << x[i] << "\", \"" << y[j] << "\", \""
    //              << path(x[i]) / y[j] << "\");\n";
    //  }

    PATH_CHECK(path("") / "", "");
    append_test_aux("", "", "");

    PATH_CHECK(path("") / "/", "/");
    append_test_aux("", "/", "/");

    PATH_CHECK(path("") / "bar", "bar");
    append_test_aux("", "bar", "bar");

    PATH_CHECK(path("") / "/bar", "/bar");
    append_test_aux("", "/bar", "/bar");

    PATH_CHECK(path("/") / "", "/");
    append_test_aux("/", "", "/");

    PATH_CHECK(path("/") / "/", "//");
    append_test_aux("/", "/", "//");

    PATH_CHECK(path("/") / "bar", "/bar");
    append_test_aux("/", "bar", "/bar");

    PATH_CHECK(path("/") / "/bar", "//bar");
    append_test_aux("/", "/bar", "//bar");

    PATH_CHECK(path("foo") / "", "foo");
    append_test_aux("foo", "", "foo");

    PATH_CHECK(path("foo") / "/", "foo/");
    append_test_aux("foo", "/", "foo/");

    PATH_CHECK(path("foo") / "/bar", "foo/bar");
    append_test_aux("foo", "/bar", "foo/bar");

    PATH_CHECK(path("foo/") / "", "foo/");
    append_test_aux("foo/", "", "foo/");

    PATH_CHECK(path("foo/") / "/", "foo//");
    append_test_aux("foo/", "/", "foo//");

    PATH_CHECK(path("foo/") / "bar", "foo/bar");
    append_test_aux("foo/", "bar", "foo/bar");

    PATH_CHECK(path("foo/") / "/bar", "foo//bar");
    append_test_aux("foo/", "/bar", "foo//bar");

    if (platform == "Windows")
    {
      PATH_CHECK(path("foo") / "bar", "foo\\bar");
      append_test_aux("foo", "bar", "foo\\bar");

      // hand created test case specific to Windows
      PATH_CHECK(path("c:") / "bar", "c:bar");
      append_test_aux("c:", "bar", "c:bar");
    }
    else
    {
      PATH_CHECK(path("foo") / "bar", "foo/bar");
      append_test_aux("foo", "bar", "foo/bar");
    }

  }

  //  name_function_tests  -------------------------------------------------------------//

  void name_function_tests()
  {
    std::cout << "name_function_tests..." << std::endl;

    BOOST_TEST(fs::portable_posix_name(std::string("x")));
    BOOST_TEST(fs::windows_name(std::string("x")));
    BOOST_TEST(fs::portable_name(std::string("x")));
    BOOST_TEST(fs::portable_directory_name(std::string("x")));
    BOOST_TEST(fs::portable_file_name(std::string("x")));

    BOOST_TEST(fs::portable_posix_name(std::string(".")));
    BOOST_TEST(fs::windows_name(std::string(".")));
    BOOST_TEST(fs::portable_name(std::string(".")));
    BOOST_TEST(fs::portable_directory_name(std::string(".")));
    BOOST_TEST(!fs::portable_file_name(std::string(".")));

    BOOST_TEST(fs::portable_posix_name(std::string("..")));
    BOOST_TEST(fs::windows_name(std::string("..")));
    BOOST_TEST(fs::portable_name(std::string("..")));
    BOOST_TEST(fs::portable_directory_name(std::string("..")));
    BOOST_TEST(!fs::portable_file_name(std::string("..")));

    BOOST_TEST(!fs::native(std::string("")));
    BOOST_TEST(!fs::portable_posix_name(std::string("")));
    BOOST_TEST(!fs::windows_name(std::string("")));
    BOOST_TEST(!fs::portable_name(std::string("")));
    BOOST_TEST(!fs::portable_directory_name(std::string("")));
    BOOST_TEST(!fs::portable_file_name(std::string("")));

    BOOST_TEST(!fs::native(std::string(" ")));
    BOOST_TEST(!fs::portable_posix_name(std::string(" ")));
    BOOST_TEST(!fs::windows_name(std::string(" ")));
    BOOST_TEST(!fs::portable_name(std::string(" ")));
    BOOST_TEST(!fs::portable_directory_name(std::string(" ")));
    BOOST_TEST(!fs::portable_file_name(std::string(" ")));

    BOOST_TEST(!fs::portable_posix_name(std::string(":")));
    BOOST_TEST(!fs::windows_name(std::string(":")));
    BOOST_TEST(!fs::portable_name(std::string(":")));
    BOOST_TEST(!fs::portable_directory_name(std::string(":")));
    BOOST_TEST(!fs::portable_file_name(std::string(":")));

    BOOST_TEST(fs::portable_posix_name(std::string("-")));
    BOOST_TEST(fs::windows_name(std::string("-")));
    BOOST_TEST(!fs::portable_name(std::string("-")));
    BOOST_TEST(!fs::portable_directory_name(std::string("-")));
    BOOST_TEST(!fs::portable_file_name(std::string("-")));

    BOOST_TEST(!fs::portable_posix_name(std::string("foo bar")));
    BOOST_TEST(fs::windows_name(std::string("foo bar")));
    BOOST_TEST(!fs::windows_name(std::string(" bar")));
    BOOST_TEST(!fs::windows_name(std::string("foo ")));
    BOOST_TEST(!fs::portable_name(std::string("foo bar")));
    BOOST_TEST(!fs::portable_directory_name(std::string("foo bar")));
    BOOST_TEST(!fs::portable_file_name(std::string("foo bar")));

    BOOST_TEST(fs::portable_posix_name(std::string("foo.bar")));
    BOOST_TEST(fs::windows_name(std::string("foo.bar")));
    BOOST_TEST(fs::portable_name(std::string("foo.bar")));
    BOOST_TEST(!fs::portable_directory_name(std::string("foo.bar")));
    BOOST_TEST(fs::portable_file_name(std::string("foo.bar")));

    BOOST_TEST(fs::portable_posix_name(std::string("foo.barf")));
    BOOST_TEST(fs::windows_name(std::string("foo.barf")));
    BOOST_TEST(fs::portable_name(std::string("foo.barf")));
    BOOST_TEST(!fs::portable_directory_name(std::string("foo.barf")));
    BOOST_TEST(!fs::portable_file_name(std::string("foo.barf")));

    BOOST_TEST(fs::portable_posix_name(std::string(".foo")));
    BOOST_TEST(fs::windows_name(std::string(".foo")));
    BOOST_TEST(!fs::portable_name(std::string(".foo")));
    BOOST_TEST(!fs::portable_directory_name(std::string(".foo")));
    BOOST_TEST(!fs::portable_file_name(std::string(".foo")));

    BOOST_TEST(fs::portable_posix_name(std::string("foo.")));
    BOOST_TEST(!fs::windows_name(std::string("foo.")));
    BOOST_TEST(!fs::portable_name(std::string("foo.")));
    BOOST_TEST(!fs::portable_directory_name(std::string("foo.")));
    BOOST_TEST(!fs::portable_file_name(std::string("foo.")));
  }
  
  //  replace_extension_tests  ---------------------------------------------------//

  void replace_extension_tests()
  {
    std::cout << "replace_extension_tests..." << std::endl;

    BOOST_TEST(path().replace_extension().empty());
    BOOST_TEST(path().replace_extension("a").empty());
    BOOST_TEST(path().replace_extension("a.") == ".");
    BOOST_TEST(path().replace_extension("a.txt") == ".txt");
    // see the rationale in html docs for explanation why this works:
    BOOST_TEST(path().replace_extension(".txt") == ".txt");

    BOOST_TEST(path("a.txt").replace_extension() == "a");
    BOOST_TEST(path("a.txt").replace_extension("") == "a");
    BOOST_TEST(path("a.txt").replace_extension(".") == "a.");
    BOOST_TEST(path("a.txt").replace_extension(".tex") == "a.tex");
    BOOST_TEST(path("a.txt").replace_extension("tex") == "a");
    BOOST_TEST(path("a.").replace_extension(".tex") == "a.tex");
    BOOST_TEST(path("a.").replace_extension("tex") == "a");
    BOOST_TEST(path("a").replace_extension(".txt") == "a.txt");
    BOOST_TEST(path("a").replace_extension("txt") == "a");
    BOOST_TEST(path("a.b.txt").replace_extension(".tex") == "a.b.tex");  
    BOOST_TEST(path("a.b.txt").replace_extension("tex") == "a.b");
    BOOST_TEST(path("a/b").replace_extension(".c") == "a/b.c");
    BOOST_TEST_EQ(path("a.txt/b").replace_extension(".c"), "a.txt/b.c"); // ticket 4702
  }
  
  //  make_preferred_tests  ------------------------------------------------------------//

  void make_preferred_tests()
  {
    std::cout << "make_preferred_tests..." << std::endl;

    if (platform == "Windows")
    {
      BOOST_TEST(path("//abc\\def/ghi").make_preferred().native()
        == path("\\\\abc\\def\\ghi").native());
    }
    else
    {
      BOOST_TEST(path("//abc\\def/ghi").make_preferred().native()
        == path("//abc\\def/ghi").native());
    }
  }

} // unnamed namespace

//--------------------------------------------------------------------------------------//
//                                                                                      //
//                                     main                                             //
//                                                                                      //
//--------------------------------------------------------------------------------------//

int cpp_main(int, char*[])
{
  // The choice of platform is make at runtime rather than compile-time
  // so that compile errors for all platforms will be detected even though
  // only the current platform is runtime tested.
  platform = (platform == "Win32" || platform == "Win64" || platform == "Cygwin")
               ? "Windows"
               : "POSIX";
  std::cout << "Platform is " << platform << '\n';

  BOOST_TEST(p1.string() != p3.string());
  p3 = p2;
  BOOST_TEST(p1.string() == p3.string());

  path p4("foobar");
  BOOST_TEST(p4.string() == "foobar");
  p4 = p4; // self-assignment
  BOOST_TEST(p4.string() == "foobar");

  construction_tests();
  append_tests();
  overload_tests();
  query_and_decomposition_tests();
  composition_tests();
  iterator_tests();
  non_member_tests();
  exception_tests();
  name_function_tests();
  replace_extension_tests();
  make_preferred_tests();

  // verify deprecated names still available

# ifndef BOOST_FILESYSTEM_NO_DEPRECATED

  p1.branch_path();
  p1.leaf();
  path p_remove_leaf;
  p_remove_leaf.remove_leaf();

# endif

  std::string s1("//:somestring");  // this used to be treated specially

  // check the path member templates
  p5.assign(s1.begin(), s1.end());

  PATH_CHECK(p5, "//:somestring");
  p5 = s1;
  PATH_CHECK(p5, "//:somestring");

  // this code, courtesy of David Whetstone, detects a now fixed bug that
  // derefereced the end iterator (assuming debug build with checked itors)
  std::vector<char> v1;
  p5.assign(v1.begin(), v1.end());
  std::string s2(v1.begin(), v1.end());
  PATH_CHECK(p5, s2);
  p5.assign(s1.begin(), s1.begin() + 1);
  PATH_CHECK(p5, "/");

  BOOST_TEST(p1 != p4);
  BOOST_TEST(p1.string() == p2.string());
  BOOST_TEST(p1.string() == p3.string());
  BOOST_TEST(path("foo").filename() == "foo");
  BOOST_TEST(path("foo").parent_path().string() == "");
  BOOST_TEST(p1.filename() == "fum");
  BOOST_TEST(p1.parent_path().string() == "fe/fi/fo");
  BOOST_TEST(path("").empty() == true);
  BOOST_TEST(path("foo").empty() == false);

  // inserter and extractor tests
# if !defined(BOOST_MSVC) || BOOST_MSVC > 1300 // bypass VC++ 7.0 and earlier
  std::cout << "\nInserter and extractor test...";
  std::stringstream ss;
  ss << fs::path("foo/bar") << std::endl;
  fs::path round_trip;
  ss >> round_trip;
  BOOST_TEST(round_trip.string() == "foo/bar");
  std::cout << round_trip.string() << "..." << round_trip << " complete\n";
# endif

  return ::boost::report_errors();
}
