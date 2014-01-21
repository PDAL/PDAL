.. _pdal_test:

=======
Testing
=======


Unit Tests
==========

A unit test framework is provided, with the goal that all (nontrivial)
classes will have unit tests.  At the very least, each new class should have
a corresponding unit test file stubbed in, even if there aren't any tests
yet.

Our unit tests also include testing of each of the command line
apps.

We use the Boost unit test framework.  This gives us automatic 
memory leak testing as well.

Unit tests for features that are configuration-dependent, e.g. laszip
compression, should be put under the same #ifdef guards as the classes
being tested.

The Support class, in the test/unit directory, provides some functions for comparing files, etc, that are useful in writing test cases.

Unit tests should not be long-running.  (As of this writing, a full test
run executes 163 tests in under 10 seconds in Debug mode on mpg's little
laptop.  This is good.)

To run the unit tests, issue the following command from your build directory::

  $ make test

Your output should resemble the following::

  Running tests...
  Test project /path/to/pdal/build
      Start 1: pdal_test
  1/1 Test #1: pdal_test ........................   Passed    3.53 sec

  100% tests passed, 0 tests failed out of 1

  Total Test time (real) =   4.79 sec

Alternately, one can run the unit tests directly::

  $ bin/pdal_test ../../test/data/ --catch_system_errors=no

Again, the output should resemble the following::

  Running 198 test cases...
  0.100
  Wrote 1065 points
  0.100
  Wrote 1065 points
  0.100
  Wrote 1065 points
  0.100
  Wrote 1065 points

  *** No errors detected

Test Data
=========

Use the directory ./test/data to store files used for unit tests.  A
function is provided in the Support class for referencing that directory
in a configuration-independent manner.

Temporary output files from unit tests should go into the test/temp directory.
A Support function is provided for referencing this directory as well.

Unit tests should always clean up and remove any files that they create
(except perhaps in case of a failed test, in which case leaving the output
around might be helpful for debugging).
