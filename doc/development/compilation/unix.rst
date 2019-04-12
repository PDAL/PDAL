.. _building_unix:

******************************************************************************
Unix Compilation
******************************************************************************

PDAL comes with support for building with `CMake`_.  PDAL requires at
least version 3.5 of CMake.
CMake is a cross-platform meta-build system that provides a unified system
for building applications on multiple platforms with various build tools.
CMake has `generators`_ for many build tools, though PDAL has been tested
only with `Ninja`_ and `GNU Makefiles`_ on Unix/OSX.  Ninja builds PDAL faster,
so the following instructions use that build tool, though building with
GNU Makefiles works similarly (simply replace "ninja" with "make" when
running the build tool).

.. _`CMake`: https://cmake.org
.. _`generators`: https://cmake.org/cmake/help/v3.5/manual/cmake-generators.7.html
.. _`Ninja`: https://ninja-build.org/
.. _`GNU Makefiles`: https://www.gnu.org/software/make/manual/make.html

Dependencies
------------------------------------------------------------------------------

Building PDAL successfully depends on having other libraries configured
and installed.  These :ref:`dependencies <dependencies>` can be built
from source or
can be installed via a packaging system (`apt`_ works well on Ubuntu and
Debian-based Linux systems. `Conda`_ works well on most systems.  Some have
had success with `brew`_ on OSX systems.)
Often, the only package that
needs to be installed prior to building PDAL is GDAL.  Installing a GDAL
package will normally install other PDAL dependencies automatically.

::

    $ apt install libgdal-dev

    OR

    $ conda install gdal

    OR

    $ brew install gdal

.. _`apt`: https://help.ubuntu.com/lts/serverguide/apt.html
.. _`Conda`: https://conda.io/en/latest/
.. _`brew`: https://brew.sh/

Using Ninja on Linux or OSX
------------------------------------------------------------------------------

Get the source code
..............................................................................

PDAL can be cloned from :ref:`GitHub <source>` or you can download a
:ref:`release bundle <download>`

Prepare a build directory
..............................................................................

CMake allows you to generate different builders for a project.  Here we're
using Mac OSX, but the procedure and output are nearly identical on Linux
distributions.

::

    $ cd PDAL
    $ mkdir build
    $ cd build

Run CMake
..............................................................................

Running CMake uses the specified generator to create
an environment suitable for building PDAL with the requested tool.
(Ninja in this case).

::

    $ cmake -G Ninja ..
    -- Could NOT find JSONCPP (missing: JSONCPP_LIBRARY JSONCPP_INCLUDE_DIR) (Required is at least version "1.6.2")
    -- Numpy output: /usr/lib/python2.7/dist-packages/numpy/core/include
    1.13.3

    -- Could NOT find LIBEXECINFO (missing: LIBEXECINFO_LIBRARY)
    -- Could NOT find LIBUNWIND (missing: LIBUNWIND_LIBRARY LIBUNWIND_INCLUDE_DIR)
    -- The following features have been enabled:

     * PostgreSQL PointCloud plugin, read/write PostgreSQL PointCloud objects
     * Python plugin, add features that depend on python
     * Unit tests, PDAL unit tests

    -- The following OPTIONAL packages have been found:

     * PkgConfig
     * LibXml2
     * Curl

    -- The following REQUIRED packages have been found:

     * GDAL (required version >= 2.2.0)
       Provides general purpose raster, vector, and reference system support
    ...
    -- The following RECOMMENDED packages have not been found:

     * LASzip (required version >= 3.1)
       Provides LASzip compression

    -- Configuring done
    -- Generating done
    -- Build files have been written to: /home/foo/pdal/build

Issue the `ninja` command
..............................................................................

If cmake runs to completion (reports that build files have been written),
you can run Ninja to build PDAL.

::

    $ ninja

If no errors are reported, Ninja will have created the ``pdal`` program
in the ``bin`` directory.  A set of necessary support libraries will have
been created in the ``lib`` directory.

::

    $ ls bin/pdal
    bin/pdal

    $ ls lib/libpdalcpp*
    lib/libpdalcpp.8.dylib
    lib/libpdalcpp.dylib
    lib/libpdalcpp.9.0.0.dylib

Checking the build and running PDAL tests
..............................................................................

You can quickly check that PDAL has built properly by running the `pdal info`
command.

::

    $ bin/pdal info ../test/data/las/autzen_trim.las
    {
      "filename": "../test/data/las/autzen_trim.las",
      "pdal_version": "1.8.0 (git-version: c39e62)",
      "stats":
      {
        "bbox":
        {
          "EPSG:4326":
          {
            "bbox":
            {
              "maxx": -123.0689038,
              "maxy": 44.0515451,
              "maxz": 158.651448,
              "minx": -123.0734481,
              "miny": 44.04990077,
              "minz": 123.828048
            },
    ...

CMake will normally build a set of tests that can be used to verify that PDAL
executes most functions properly.  You can run these tests yourself if
desired, though it's not typically necessary.

::

    $ ctest
    Test project /Users/foo/pdal.master/build
          Start  1: pdal_filters_pcl_block_test
     1/97 Test  #1: pdal_filters_pcl_block_test ............   Passed    0.23 sec
          Start  2: pdal_filters_icp_test
     2/97 Test  #2: pdal_filters_icp_test ..................   Passed    0.12 sec
          Start  3: pdal_filters_python_test
     3/97 Test  #3: pdal_filters_python_test ...............   Passed    3.52 sec
          Start  4: pdal_io_numpy_test
     4/97 Test  #4: pdal_io_numpy_test .....................   Passed    0.31 sec
      ...
    93/96 Test #93: pdal_io_ilvis2_metadata_test ...........   Passed    0.03 sec
          Start 94: pdal_io_ilvis2_reader_metadata_test
    94/96 Test #94: pdal_io_ilvis2_reader_metadata_test ....   Passed    0.05 sec
          Start 95: xml_schema_test
    95/96 Test #95: xml_schema_test ........................   Passed    0.04 sec
          Start 96: pdal_io_ilvis2_test
    96/96 Test #96: pdal_io_ilvis2_test ....................   Passed    0.04 sec

    100% tests passed, 0 tests failed out of 96

    Total Test time (real) =  39.54 sec

Failed tests may not indicate problems other than a lack of support for some
feature on your system.  For example, tests for database drivers will fail if
the database isn't installed or configured properly.

Install PDAL
..............................................................................

PDAL can be installed to the default location (usually subdirectories of
/usr/local) using Ninja.

::

    $ ninja install
