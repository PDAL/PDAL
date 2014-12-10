.. _pdal_test:

=======
Testing
=======


Unit Tests
==========

A unit test framework is provided, with the goal that all (nontrivial) classes
will have unit tests.  At the very least, each new class should have a
corresponding unit test file stubbed in, even if there aren't any tests yet.

Our unit tests also include testing of each of the command line apps and
(known) plugins.

We use the Google C++ Test Framework.

Unit tests for features that are configuration-dependent, e.g. laszip
compression, should be put under the same ``#ifdef`` guards as the classes
being tested.

The Support class, in the ``./test/unit`` directory, provides some functions
for comparing files, etc, that are useful in writing test cases.

Unit tests should not be long-running. (As of this writing, a full test run
executes 163 tests in under 10 seconds in Debug mode on mpg's little laptop.
This is good.)

To run all unit tests, issue the following command from your build directory::

  $ ctest

``make test`` or ``ninja test`` should still work as well.

Depending on the which optional components you've chose to build, your output
should resemble the following::

  Test project /path/to/pdal/build
        Start  1: pdal_base_test
   1/10 Test  #1: pdal_base_test ...................   Passed    3.13 sec
        Start  2: pdal_io_test
   2/10 Test  #2: pdal_io_test .....................   Passed    1.19 sec
        Start  3: pdal_filters_test
   3/10 Test  #3: pdal_filters_test ................   Passed    6.27 sec
        Start  4: pdal_plang_test
   4/10 Test  #4: pdal_plang_test ..................   Passed    1.32 sec
        Start  5: pc2pc_test
   5/10 Test  #5: pc2pc_test .......................   Passed    5.62 sec
        Start  6: xml_schema_test
   6/10 Test  #6: xml_schema_test ..................   Passed    0.02 sec
        Start  7: hexbintest
   7/10 Test  #7: hexbintest .......................   Passed    1.15 sec
        Start  8: icetest
   8/10 Test  #8: icetest ..........................   Passed    1.12 sec
        Start  9: nitftest
   9/10 Test  #9: nitftest .........................   Passed    1.28 sec
        Start 10: pcltest
  10/10 Test #10: pcltest ..........................   Passed    1.47 sec

  100% tests passed, 0 tests failed out of 10

  Total Test time (real) =  22.61 sec

For a more verbose output, use the ``-V`` flag. Or, to run an individual test
suite, use ``-R <suite name>``. For example::

  $ ctest -V -R pdal_io_test

Should produce output similar to::

  UpdateCTestConfiguration  from :/path/to/pdal/build/DartConfiguration.tcl
  UpdateCTestConfiguration  from :/path/to/pdal/build/DartConfiguration.tcl
  Test project /path/to/pdal/build
  Constructing a list of tests
  Done constructing a list of tests
  Checking test dependency graph...
  Checking test dependency graph end
  test 2
      Start 2: pdal_io_test

  2: Test command: /path/to/pdal/build/bin/pdal_io_test
  2: Test timeout computed to be: 9.99988e+06
  2: Running main() from gtest_main.cc
  2: [==========] Running 28 tests from 8 test cases.
  2: [----------] Global test environment set-up.
  2: [----------] 7 tests from BPFTest
  2: [ RUN      ] BPFTest.test_point_major
  2: [       OK ] BPFTest.test_point_major (8 ms)
  2: [ RUN      ] BPFTest.test_dim_major
  2: [       OK ] BPFTest.test_dim_major (2 ms)
  2: [ RUN      ] BPFTest.test_byte_major
  2: [       OK ] BPFTest.test_byte_major (3 ms)
  2: [ RUN      ] BPFTest.test_point_major_zlib
  2: [       OK ] BPFTest.test_point_major_zlib (4 ms)
  2: [ RUN      ] BPFTest.test_dim_major_zlib
  2: [       OK ] BPFTest.test_dim_major_zlib (3 ms)
  2: [ RUN      ] BPFTest.test_byte_major_zlib
  2: [       OK ] BPFTest.test_byte_major_zlib (4 ms)
  2: [ RUN      ] BPFTest.inspect
  2: [       OK ] BPFTest.inspect (0 ms)
  2: [----------] 7 tests from BPFTest (24 ms total)
  2:
         ...
  2:
  2: [----------] Global test environment tear-down
  2: [==========] 28 tests from 8 test cases ran. (1195 ms total)
  2: [  PASSED  ] 28 tests.
  1/1 Test #2: pdal_io_test .....................   Passed    1.24 sec

  The following tests passed:
          pdal_io_test

  100% tests passed, 0 tests failed out of 1

  Total Test time (real) =   1.24 sec

Alternately, one can run individual unit tests directly::

  $ bin/pdal_io_test

Again, the output should resemble the following::

  Running main() from gtest_main.cc
  [==========] Running 28 tests from 8 test cases.
  [----------] Global test environment set-up.
  [----------] 7 tests from BPFTest
  [ RUN      ] BPFTest.test_point_major
  [       OK ] BPFTest.test_point_major (5 ms)
  [ RUN      ] BPFTest.test_dim_major
  [       OK ] BPFTest.test_dim_major (2 ms)
  [ RUN      ] BPFTest.test_byte_major
  [       OK ] BPFTest.test_byte_major (3 ms)
  [ RUN      ] BPFTest.test_point_major_zlib
  [       OK ] BPFTest.test_point_major_zlib (4 ms)
  [ RUN      ] BPFTest.test_dim_major_zlib
  [       OK ] BPFTest.test_dim_major_zlib (3 ms)
  [ RUN      ] BPFTest.test_byte_major_zlib
  [       OK ] BPFTest.test_byte_major_zlib (3 ms)
  [ RUN      ] BPFTest.inspect
  [       OK ] BPFTest.inspect (0 ms)
  [----------] 7 tests from BPFTest (21 ms total)

      ...

  [----------] Global test environment tear-down
  [==========] 28 tests from 8 test cases ran. (1180 ms total)
  [  PASSED  ] 28 tests.

This invocation allows us to alter Google Test's default behavior. For more on
the available flags type::

  $ bin/<test_name> --help

Key among these flags are the ability to list tests (``--gtest_list_tests``)
and to run only select tests (``--gtest_filter``).

Test Data
=========

Use the directory ``./test/data`` to store files used for unit tests.  A
vfunction is provided in the Support class for referencing that directory in a
configuration-independent manner.

Temporary output files from unit tests should go into the ``./test/temp``
directory.  A Support function is provided for referencing this directory as
well.

Unit tests should always clean up and remove any files that they create (except
perhaps in case of a failed test, in which case leaving the output around might
be helpful for debugging).
