.. _pdal_test:

================================================================================
Testing
================================================================================


Unit Tests
================================================================================

A unit test framework is provided, with the goal that all (nontrivial) classes
will have unit tests.  At the very least, each new class should have a
corresponding unit test file stubbed in, even if there aren't any tests yet.

*   Our unit tests also include testing of the command line :ref:`apps` and
    (known) :ref:`plugins <stage_index>`.

*    We use the `Google C++ Test Framework`_, but a local copy of it is
     embedded in the PDAL source tree, and you don't have to have it available
     as a dependency.

.. _`Google C++ Test Framework`: https://code.google.com/p/googletest/

*    Unit tests for features that are configuration-dependent, e.g. laszip
     compression, should be put under the same ``#ifdef`` guards as the classes
     being tested.

*    The Support class, in the ``./test/unit`` directory, provides some functions
     for comparing files, etc, that are useful in writing test cases.

*    Unit tests should not be long-running.

Running the Tests
================================================================================

To run all unit tests, issue the following command from your build directory::

  $ ctest

``make test`` or ``ninja test`` should still work as well.

Depending on the which optional components you've chose to build, your output
should resemble the following::

    Test project /Users/hobu/dev/git/pdal
          Start  1: pdal_bounds_test
     1/61 Test  #1: pdal_bounds_test ...................   Passed    0.02 sec
          Start  2: pdal_config_test
     2/61 Test  #2: pdal_config_test ...................   Passed    0.02 sec
          Start  3: pdal_file_utils_test
     3/61 Test  #3: pdal_file_utils_test ...............   Passed    0.02 sec
          Start  4: pdal_georeference_test
     4/61 Test  #4: pdal_georeference_test .............   Passed    0.02 sec
          Start  5: pdal_kdindex_test
     5/61 Test  #5: pdal_kdindex_test ..................   Passed    0.03 sec
          Start  6: pdal_log_test
     6/61 Test  #6: pdal_log_test ......................   Passed    0.03 sec
          Start  7: pdal_metadata_test
     7/61 Test  #7: pdal_metadata_test .................   Passed    0.02 sec
          Start  8: pdal_options_test
     8/61 Test  #8: pdal_options_test ..................   Passed    0.02 sec
          Start  9: pdal_pdalutils_test
     9/61 Test  #9: pdal_pdalutils_test ................   Passed    0.02 sec
          Start 10: pdal_pipeline_manager_test
    10/61 Test #10: pdal_pipeline_manager_test .........   Passed    0.03 sec
          Start 11: pdal_point_view_test
    11/61 Test #11: pdal_point_view_test ...............   Passed    2.03 sec
          Start 12: pdal_point_table_test
    12/61 Test #12: pdal_point_table_test ..............   Passed    0.03 sec
          Start 13: pdal_spatial_reference_test
    13/61 Test #13: pdal_spatial_reference_test ........   Passed    0.07 sec
          Start 14: pdal_support_test
    14/61 Test #14: pdal_support_test ..................   Passed    0.02 sec
          Start 15: pdal_user_callback_test
    15/61 Test #15: pdal_user_callback_test ............   Passed    0.02 sec
          Start 16: pdal_utils_test
    16/61 Test #16: pdal_utils_test ....................   Passed    0.02 sec
          Start 17: pdal_lazperf_test
    17/61 Test #17: pdal_lazperf_test ..................   Passed    0.04 sec
          Start 18: pdal_io_bpf_test
    18/61 Test #18: pdal_io_bpf_test ...................   Passed    0.20 sec
          Start 19: pdal_io_buffer_test
    19/61 Test #19: pdal_io_buffer_test ................   Passed    0.02 sec
          Start 20: pdal_io_faux_test
    20/61 Test #20: pdal_io_faux_test ..................   Passed    0.04 sec
          Start 21: pdal_io_ilvis2_test
    21/61 Test #21: pdal_io_ilvis2_test ................   Passed    0.06 sec
          Start 22: pdal_io_las_reader_test
    22/61 Test #22: pdal_io_las_reader_test ............   Passed    0.49 sec
          Start 23: pdal_io_las_writer_test
    23/61 Test #23: pdal_io_las_writer_test ............   Passed    2.27 sec
          Start 24: pdal_io_optech_test
    24/61 Test #24: pdal_io_optech_test ................   Passed    0.03 sec
          Start 25: pdal_io_ply_reader_test
    25/61 Test #25: pdal_io_ply_reader_test ............   Passed    0.03 sec
          Start 26: pdal_io_ply_writer_test
    26/61 Test #26: pdal_io_ply_writer_test ............   Passed    0.02 sec
          Start 27: pdal_io_qfit_test
    27/61 Test #27: pdal_io_qfit_test ..................   Passed    0.03 sec
          Start 28: pdal_io_sbet_reader_test
    28/61 Test #28: pdal_io_sbet_reader_test ...........   Passed    0.04 sec
          Start 29: pdal_io_sbet_writer_test
    29/61 Test #29: pdal_io_sbet_writer_test ...........   Passed    0.03 sec
          Start 30: pdal_io_terrasolid_test
    30/61 Test #30: pdal_io_terrasolid_test ............   Passed    0.03 sec
          Start 31: pdal_filters_chipper_test
    31/61 Test #31: pdal_filters_chipper_test ..........   Passed    0.03 sec
          Start 32: pdal_filters_colorization_test
    32/61 Test #32: pdal_filters_colorization_test .....   Passed   11.40 sec
          Start 33: pdal_filters_crop_test
    33/61 Test #33: pdal_filters_crop_test .............   Passed    0.04 sec
          Start 34: pdal_filters_decimation_test
    34/61 Test #34: pdal_filters_decimation_test .......   Passed    0.02 sec
          Start 35: pdal_filters_divider_test
    35/61 Test #35: pdal_filters_divider_test ..........   Passed    0.03 sec
          Start 36: pdal_filters_ferry_test
    36/61 Test #36: pdal_filters_ferry_test ............   Passed    0.04 sec
          Start 37: pdal_filters_merge_test
    37/61 Test #37: pdal_filters_merge_test ............   Passed    0.03 sec
          Start 38: pdal_filters_reprojection_test
    38/61 Test #38: pdal_filters_reprojection_test .....   Passed    0.03 sec
          Start 39: pdal_filters_range_test
    39/61 Test #39: pdal_filters_range_test ............   Passed    0.05 sec
          Start 40: pdal_filters_randomize_test
    40/61 Test #40: pdal_filters_randomize_test ........   Passed    0.02 sec
          Start 41: pdal_filters_sort_test
    41/61 Test #41: pdal_filters_sort_test .............   Passed    0.39 sec
          Start 42: pdal_filters_splitter_test
    42/61 Test #42: pdal_filters_splitter_test .........   Passed    0.03 sec
          Start 43: pdal_filters_stats_test
    43/61 Test #43: pdal_filters_stats_test ............   Passed    0.03 sec
          Start 44: pdal_filters_transformation_test
    44/61 Test #44: pdal_filters_transformation_test ...   Passed    0.03 sec
          Start 45: pdal_merge_test
    45/61 Test #45: pdal_merge_test ....................   Passed    0.07 sec
          Start 46: pc2pc_test
    46/61 Test #46: pc2pc_test .........................   Passed    0.15 sec
          Start 47: xml_schema_test
    47/61 Test #47: xml_schema_test ....................   Passed    0.02 sec
          Start 48: pdal_filters_attribute_test
    48/61 Test #48: pdal_filters_attribute_test ........   Passed    0.09 sec
          Start 49: pdal_plugins_cpd_kernel_test
    49/61 Test #49: pdal_plugins_cpd_kernel_test .......***Exception: Other  0.08 sec
          Start 50: hexbintest
    50/61 Test #50: hexbintest .........................   Passed    0.03 sec
          Start 51: icetest
    51/61 Test #51: icetest ............................   Passed    0.04 sec
          Start 52: mrsidtest
    52/61 Test #52: mrsidtest ..........................   Passed    0.06 sec
          Start 53: pdal_io_nitf_writer_test
    53/61 Test #53: pdal_io_nitf_writer_test ...........   Passed    0.08 sec
          Start 54: pdal_io_nitf_reader_test
    54/61 Test #54: pdal_io_nitf_reader_test ...........   Passed    0.04 sec
          Start 55: ocitest
    55/61 Test #55: ocitest ............................***Failed    0.06 sec
          Start 56: pcltest
    56/61 Test #56: pcltest ............................   Passed    0.28 sec
          Start 57: pgpointcloudtest
    57/61 Test #57: pgpointcloudtest ...................   Passed    1.66 sec
          Start 58: plangtest
    58/61 Test #58: plangtest ..........................   Passed    0.14 sec
          Start 59: python_predicate_test
    59/61 Test #59: python_predicate_test ..............   Passed    0.16 sec
          Start 60: python_programmable_test
    60/61 Test #60: python_programmable_test ...........   Passed    0.15 sec
          Start 61: sqlitetest
    61/61 Test #61: sqlitetest .........................   Passed    0.55 sec

    97% tests passed, 2 tests failed out of 61

    Total Test time (real) =  21.57 sec

    The following tests FAILED:
         49 - pdal_plugins_cpd_kernel_test (OTHER_FAULT)
         55 - ocitest (Failed)

For a more verbose output, use the ``-V`` flag. Or, to run an individual test
suite, use ``-R <suite name>``. For example::

  $ ctest -V -R pdal_io_bpf_test

Should produce output similar to::

    UpdateCTestConfiguration  from :/Users/hobu/dev/git/pdal/DartConfiguration.tcl
    UpdateCTestConfiguration  from :/Users/hobu/dev/git/pdal/DartConfiguration.tcl
    Test project /Users/hobu/dev/git/pdal
    Constructing a list of tests
    Done constructing a list of tests
    Checking test dependency graph...
    Checking test dependency graph end
    test 18
        Start 18: pdal_io_bpf_test

    18: Test command: /Users/hobu/dev/git/pdal/bin/pdal_io_bpf_test
    18: Environment variables:
    18:  PDAL_DRIVER_PATH=/Users/hobu/dev/git/pdal/lib
    18: Test timeout computed to be: 9.99988e+06
    18: [==========] Running 20 tests from 1 test case.
    18: [----------] Global test environment set-up.
    18: [----------] 20 tests from BPFTest
    18: [ RUN      ] BPFTest.test_point_major
    18: [       OK ] BPFTest.test_point_major (8 ms)
    18: [ RUN      ] BPFTest.test_dim_major
    18: [       OK ] BPFTest.test_dim_major (3 ms)
    18: [ RUN      ] BPFTest.test_byte_major
    18: [       OK ] BPFTest.test_byte_major (4 ms)
    18: [ RUN      ] BPFTest.test_point_major_zlib
    18: [       OK ] BPFTest.test_point_major_zlib (6 ms)
    18: [ RUN      ] BPFTest.test_dim_major_zlib
    18: [       OK ] BPFTest.test_dim_major_zlib (4 ms)
    18: [ RUN      ] BPFTest.test_byte_major_zlib
    18: [       OK ] BPFTest.test_byte_major_zlib (5 ms)
    18: [ RUN      ] BPFTest.roundtrip_byte
    18: [       OK ] BPFTest.roundtrip_byte (15 ms)
    18: [ RUN      ] BPFTest.roundtrip_dimension
    18: [       OK ] BPFTest.roundtrip_dimension (10 ms)
    18: [ RUN      ] BPFTest.roundtrip_point
    18: [       OK ] BPFTest.roundtrip_point (11 ms)
    18: [ RUN      ] BPFTest.roundtrip_byte_compression
    18: [       OK ] BPFTest.roundtrip_byte_compression (16 ms)
    18: [ RUN      ] BPFTest.roundtrip_dimension_compression
    18: [       OK ] BPFTest.roundtrip_dimension_compression (13 ms)
    18: [ RUN      ] BPFTest.roundtrip_point_compression
    18: [       OK ] BPFTest.roundtrip_point_compression (14 ms)
    18: [ RUN      ] BPFTest.roundtrip_scaling
    18: [       OK ] BPFTest.roundtrip_scaling (10 ms)
    18: [ RUN      ] BPFTest.extra_bytes
    18: [       OK ] BPFTest.extra_bytes (15 ms)
    18: [ RUN      ] BPFTest.bundled
    18: [       OK ] BPFTest.bundled (17 ms)
    18: [ RUN      ] BPFTest.inspect
    18: [       OK ] BPFTest.inspect (1 ms)
    18: [ RUN      ] BPFTest.mueller
    18: [       OK ] BPFTest.mueller (0 ms)
    18: [ RUN      ] BPFTest.flex
    18: [       OK ] BPFTest.flex (9 ms)
    18: [ RUN      ] BPFTest.flex2
    18: [       OK ] BPFTest.flex2 (7 ms)
    18: [ RUN      ] BPFTest.outputdims
    18: [       OK ] BPFTest.outputdims (14 ms)
    18: [----------] 20 tests from BPFTest (182 ms total)
    18:
    18: [----------] Global test environment tear-down
    18: [==========] 20 tests from 1 test case ran. (182 ms total)
    18: [  PASSED  ] 20 tests.
    1/1 Test #18: pdal_io_bpf_test .................   Passed    0.20 sec

    The following tests passed:
        pdal_io_bpf_test

    100% tests passed, 0 tests failed out of 1


  $ bin/pdal_io_test

Again, the output should resemble the following::

    [==========] Running 20 tests from 1 test case.
    [----------] Global test environment set-up.
    [----------] 20 tests from BPFTest
    [ RUN      ] BPFTest.test_point_major
    [       OK ] BPFTest.test_point_major (7 ms)
    [ RUN      ] BPFTest.test_dim_major
    [       OK ] BPFTest.test_dim_major (3 ms)
    [ RUN      ] BPFTest.test_byte_major
    [       OK ] BPFTest.test_byte_major (4 ms)
    [ RUN      ] BPFTest.test_point_major_zlib
    [       OK ] BPFTest.test_point_major_zlib (5 ms)
    [ RUN      ] BPFTest.test_dim_major_zlib
    [       OK ] BPFTest.test_dim_major_zlib (5 ms)
    [ RUN      ] BPFTest.test_byte_major_zlib
    [       OK ] BPFTest.test_byte_major_zlib (6 ms)
    [ RUN      ] BPFTest.roundtrip_byte
    [       OK ] BPFTest.roundtrip_byte (17 ms)
    [ RUN      ] BPFTest.roundtrip_dimension
    [       OK ] BPFTest.roundtrip_dimension (10 ms)
    [ RUN      ] BPFTest.roundtrip_point
    [       OK ] BPFTest.roundtrip_point (11 ms)
    [ RUN      ] BPFTest.roundtrip_byte_compression
    [       OK ] BPFTest.roundtrip_byte_compression (15 ms)
    [ RUN      ] BPFTest.roundtrip_dimension_compression
    [       OK ] BPFTest.roundtrip_dimension_compression (14 ms)
    [ RUN      ] BPFTest.roundtrip_point_compression
    [       OK ] BPFTest.roundtrip_point_compression (14 ms)
    [ RUN      ] BPFTest.roundtrip_scaling
    [       OK ] BPFTest.roundtrip_scaling (11 ms)
    [ RUN      ] BPFTest.extra_bytes
    [       OK ] BPFTest.extra_bytes (16 ms)
    [ RUN      ] BPFTest.bundled
    [       OK ] BPFTest.bundled (17 ms)
    [ RUN      ] BPFTest.inspect
    [       OK ] BPFTest.inspect (1 ms)
    [ RUN      ] BPFTest.mueller
    [       OK ] BPFTest.mueller (0 ms)
    [ RUN      ] BPFTest.flex
    [       OK ] BPFTest.flex (8 ms)
    [ RUN      ] BPFTest.flex2
    [       OK ] BPFTest.flex2 (7 ms)
    [ RUN      ] BPFTest.outputdims
    [       OK ] BPFTest.outputdims (14 ms)
    [----------] 20 tests from BPFTest (185 ms total)

    [----------] Global test environment tear-down
    [==========] 20 tests from 1 test case ran. (185 ms total)
    [  PASSED  ] 20 tests.

This invocation allows us to alter Google Test's default behavior. For more on
the available flags type::

  $ bin/<test_name> --help

Key among these flags are the ability to list tests (``--gtest_list_tests``)
and to run only select tests (``--gtest_filter``).

.. note::

    If the PostgreSQL PointCloud plugin was enabled on the CMake command line
    (with ``-DBUILD_PLUGIN_PGPOINTCLOUD=ON``) then ``ctest`` will attempt to run
    the ``pgpointcloud`` tests. And you will get PostgreSQL connection errors
    if the `libpq environment variables`_ are not correctly set in your shell.
    This is for example how you can run the ``pgpointcloud`` tests::

        $ PGUSER=pdal PGPASSWORD=pdal PGHOST=localhost ctest -R pgpointcloudtest

.. _`libpq environment variables`: https://www.postgresql.org/docs/current/static/libpq-envars.html

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
