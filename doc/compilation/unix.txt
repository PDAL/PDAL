.. _building_unix:

******************************************************************************
Unix Compilation
******************************************************************************

:Author: Howard Butler
:Contact: howard@hobu.co
:Date: 11/16/2013

`CMake`_ 2.8.6+ is the prescribed tool for building from source as well as
running unit tests. `CMake`_ is a cross-platform build system that provides a
number of benefits, and its usage ensures a single, up-to-date build system
for all PDAL-supported operating systems and compiler platforms.

Like a combination of autoconf/autotools, except that it actually works on
Windows (and works without eye-stabbing pain in general), `CMake`_ is somewhat
of a meta-building tool. It can be used to generate MSVC project files, GNU
Makefiles, NMake files for MSVC, XCode projects on Mac OS X, and Eclipse
projects (as well as many others).  This functionality allows the PDAL project 
to avoid maintaining these build options by hand and target a single configuration
and build platform.

This tutorial will describe how to build PDAL using CMake on a 
Unix platform.  PDAL is known to compile on Linux 2.6's of various flavors and 
OSX Snow Leopard and Lion with XCode 4.2.

:ref:`dependencies` contains more information about specific library version
requirements and notes about building or acquiring them.

Using "Unix Makefiles" on Linux
..............................................................................

Get the source code
------------------------------------------------------------------------------

See :ref:`source` for how to obtain the latest development version or visit
:ref:`download` to get the latest released version.

Prepare a build directory
------------------------------------------------------------------------------

CMake allows you to generate different builders for a project, and in this
example, we are going to generate a "Unix Makefiles" builder
for PDAL on Mac OS X.

::
    
    $ cd PDAL
    $ mkdir makefiles
    $ cd makefiles

Configure base library
------------------------------------------------------------------------------

Configure the basic core library for the "Unix Makefiles" target:

::

    $ cmake -G "Unix Makefiles" ../
    -- The C compiler identification is GNU
    -- The CXX compiler identification is GNU
    -- Checking whether C compiler has -isysroot
    -- Checking whether C compiler has -isysroot - yes
    -- Check for working C compiler: /usr/bin/gcc
    -- Check for working C compiler: /usr/bin/gcc -- works
    -- Detecting C compiler ABI info
    -- Detecting C compiler ABI info - done
    -- Checking whether CXX compiler has -isysroot
    -- Checking whether CXX compiler has -isysroot - yes
    -- Check for working CXX compiler: /usr/bin/c++
    -- Check for working CXX compiler: /usr/bin/c++ -- works
    -- Detecting CXX compiler ABI info
    -- Detecting CXX compiler ABI info - done
    -- Enable PDAL utilities to build - done
    -- Configuring done
    -- Generating done
    -- Build files have been written to: /Users/hobu/dev/git/PDAL-cmake/makefiles


.. note::

    The ``hobu-config.sh`` shell script contains a number of common 
    settings that I use to configure my `Homebrew`-based Macintosh 
    system.

.. _`Homebrew`: http://brew.sh/
 
Issue the `make` command
------------------------------------------------------------------------------

This will build a base build of the library, with no extra libraries being
configured.


.. _make_install:

Run ``make install`` and test your installation with a :ref:`pdal_test` command
-------------------------------------------------------------------------------

``make install`` will install the :ref:`utilities <apps>` in the location
that was specified for 'CMAKE_INSTALL_PREFIX'.  Once installed, ensure that 
you can run `pdal info`.


.. _configure_optional_libraries:

Configure your :ref:`Optional Libraries <dependencies>`.
------------------------------------------------------------------------------

By checking the "on" button for each, CMake may find your installations of
these libraries, but in case it does not, set the following variables,
substituting accordingly, to values that match your system layout.

.. csv-table::

    "`GDAL`_","GDAL_CONFIG", "/usr/local/bin/gdal-config"
    "","GDAL_INCLUDE_DIR", "/usr/local/include"
    "","GDAL_LIBRARY", "/usr/local/lib/libgdal.so"
    "`GeoTIFF`_","GEOTIFF_INCLUDE_DIR","/usr/local/include"
    "","GEOTIFF_LIBRARY","/usr/local/lib/libgeotiff.so"
    "`OCI`_","ORACLE_INCLUDE_DIR","/home/oracle/sdk/include"
    "","ORACLE_NNZ_LIBRARY","/home/oracle/libnnz10.so"
    "","ORACLE_OCCI_LIBRARY","/home/oracle/libocci.so"
    "","ORACLE_OCIEI_LIBRARY","/home/oracle/libociei.so"
    "","ORACLE_OCI_LIBRARY","/home/oracle/libclntsh.so"

.. _GDAL: http://www.gdal.org
.. _Proj.4: http://trac.osgeo.org/proj
.. _GeoTIFF: http://trac.osgeo.org/geotiff
.. _libxml2: http://xmlsoft.org
.. _`OCI`: http://www.oracle.com/technology/tech/oci/index.html
.. _`Oracle Instant Client`: http://www.oracle.com/technology/tech/oci/instantclient/index.html
.. _`Oracle Point Cloud`: http://download.oracle.com/docs/cd/B28359_01/appdev.111/b28400/sdo_pc_pkg_ref.htm
.. _`DebianGIS`: http://wiki.debian.org/DebianGis
.. _`Debian`: http://www.debian.org
.. _`KyngChaos`: http://www.kyngchaos.com/software/unixport
.. _`OSGeo4W`: http://trac.osgeo.org/osgeo4w/  


CCMake and cmake-gui
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. warning::
    
    The following was just swiped from the libLAS compilation document 
    and it has not been updated for PDAL. The basics should be the same, however. 
    Please ask on the :ref:`mailing list<community>` if you run into any issues.
    
While `CMake`_ can be run from the command-line, and this is the preferred
way for many individuals, it can be much easier to run CMake from a GUI.
Now that we have a basic library building, we will use CMake's GUIs to
help us configure the rest of the optional components of the library. Run
``ccmake ../`` for the `Curses`_ interface or ``cmake-gui ../`` for a GUI
version.

  
.. figure:: media/ccmake-osx-start.png
    :alt: Running CCMake in OS X

    Running the `Curses`_ `CMake`_ interface.  This interface is available to 
    all unix-like operating systems. 
    
.. note::
    If your arrow keys are not working with in CCMake, use CTRL-N and 
    CTRL-P to move back and forth between the options.

.. figure:: media/cmake-gui-osx-start.png
    :alt: Running cmake-gui in OS X

    Running the cmake-gui `CMake`_ interface.  This interface is available 
    on Linux, Windows, and Mac OS X.

Build and install 
------------------------------------------------------------------------------

Once you have configured your additional libraries, you can install the 
software.  The main pieces that will be installed are:

* PDAL headers (typically in a location ./include/pdal/...)
* PDAL C++ (PDAL.a or PDAL.so) library
* PDAL C (PDAL_c.a or PDAL_c.so) library
* :ref:`Utility <apps>` programs

::

    make install

Using "XCode" on OS X
..............................................................................


Get the source code
------------------------------------------------------------------------------

See :ref:`source` for how to obtain the latest development version or visit
:ref:`download` to get the latest released version.

Prepare a build directory
------------------------------------------------------------------------------

CMake allows you to generate different builders for a project, and in this
example, we are going to generate an "Xcode" builder for PDAL on Mac OS X.
Additionally, we're going to use an alternative compiler -- `LLVM`_ -- which 
under certain situations can produce much faster code on Mac OS X.

::

    $ export CC=/usr/bin/llvm-gcc
    $ export CXX=/usr/bin/llvm-g++
    $ cd PDAL
    $ mkdir xcode
    $ cd xcode/

Configure base library
------------------------------------------------------------------------------

Configure the basic core library for the Xcode build:

::

    $ cmake -G "Xcode" ..
    -- The C compiler identification is GNU
    -- The CXX compiler identification is GNU
    -- Checking whether C compiler has -isysroot
    -- Checking whether C compiler has -isysroot - yes
    -- Check for working C compiler: /usr/bin/llvm-gcc
    -- Check for working C compiler: /usr/bin/llvm-gcc -- works
    -- Detecting C compiler ABI info
    -- Detecting C compiler ABI info - done
    -- Checking whether CXX compiler has -isysroot
    -- Checking whether CXX compiler has -isysroot - yes
    -- Check for working CXX compiler: /usr/bin/llvm-g++
    -- Check for working CXX compiler: /usr/bin/llvm-g++ -- works
    -- Detecting CXX compiler ABI info
    -- Detecting CXX compiler ABI info - done
    -- Enable PDAL utilities to build - done
    -- Enable PDAL unit tests to build - done
    -- Configuring done
    -- Generating done
    -- Build files have been written to: /Users/hobu/hg/PDAL-cmake/xcode


Alternatively, if you have `KyngChaos`_ frameworks for `GDAL`_ and
`GeoTIFF`_ installed, you can provide locations for those as part of your
``cmake`` invocation:

::

    $ cmake -G "Xcode" \
      -D GDAL_CONFIG=/Library/Frameworks/GDAL.framework/Programs/gdal-config \
      -D GEOTIFF_INCLUDE_DIR=/Library/Frameworks/UnixImageIO.framework/unix/include \
      -D GEOTIFF_LIBRARY=/Library/Frameworks/UnixImageIO.framework/unix/lib/libgeotiff.dylib \
      ..

    
:: 
    
    $ open PDAL.xcodeproj/

.. figure:: media/xcode-start.png
    :alt: Building PDAL using the XCode project

Set default command for XCode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Set the default executable for the project to be ``lasinfo`` by opening the 
"Executables" tree, choosing "lasinfo," and clicking the bubble next to 
the "Executable name" in the right-hand panel.  

.. figure:: media/xcode-set-default-executable.png
    :alt: Setting the default executable

Set arguments for :ref:`pdal_test`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Set the arguments for :ref:`pdal_test` so it can be run from within XCode. We
use the ${PROJECT_DIR} environment variable to be able to tell pdal_test the
location of our test file. This is similar to the :ref:`same command
<make_install>` above in the "Unix Makefiles" section.

.. figure:: media/xcode-lasinfo-arguments.png
    :alt: Setting the arguments for lasinfo



Configure :ref:`Optional Libraries <dependencies>`
------------------------------------------------------------------------------

As :ref:`before <configure_optional_libraries>`, use ``ccmake ../`` or ``cmake-gui ../`` to 
configure your :ref:`dependencies`.


.. figure:: media/cmake-gui-osx-configured.png
    :alt: Configuring optional libraries with CMake GUI
    
  
.. _`CMake`: http://www.cmake.org/
.. _`CTest`: http://cmake.org/cmake/help/ctest-2-8-docs.html
.. _`CMake 2.8.0+`: http://www.cmake.org/cmake/help/cmake-2-8-docs.html
.. _`CDash`: http://www.cdash.org/
.. _`continuous integration`: http://en.wikipedia.org/wiki/Continuous_integration
.. _`PDAL CDash`: http://my.cdash.org/index.php?project=PDAL
.. _`Curses`: http://en.wikipedia.org/wiki/Curses_%28programming_library%29
.. _`Autoconf`: http://www.gnu.org/software/autoconf/
.. _`LLVM`: http://llvm.org/
.. _`OSGeo4W`: http://trac.osgeo.org/osgeo4w/
.. _`Boost`: http://www.boost.org/

