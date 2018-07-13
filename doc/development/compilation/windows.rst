.. _building_windows:

==============================================================================
Building Under Windows
==============================================================================

:Author: Howard Butler
:Contact: howard at hobu.co
:Date: 11/02/2017

.. note::

    `OSGeo4W`_ contains a pre-built up-to-date 64 bit Windows binary. It
    is fully-featured, and if you do not need anything custom, it is likely
    the fastest way to get going.

.. _`OSGeo4W`: https://trac.osgeo.org/osgeo4w/

Introduction
------------------------------------------------------------------------------

Pre-built binary packages for Windows are available via `OSGeo4W`_ (64-bit version),
and all of the prerequisites required for compilation of a fully featured build
are also available via that packaging system. This document assumes you
will be using OSGeo4W as your base, and anything more advanced is beyond
the scope of the document.

.. note::

    The AppVeyor build system uses the PDAL project's configuration on the OSGeo4W
    system. It contains a rich resource of known working examples. See
    https://github.com/PDAL/PDAL/blob/master/appveyor.yml and
    https://github.com/PDAL/PDAL/tree/master/scripts/appveyor for inspiration.

Required Compiler
------------------------------------------------------------------------------

PDAL is known to compile on `Visual Studio 2015`_, and 2013 *might* work with
some source tree adjustments. PDAL makes heavy use of C++11, and a compiler
with good support for those features is required.

.. _`Visual Studio 2015`: https://www.visualstudio.com/vs/older-downloads/


.. _CMake: http://www.cmake.org

Prerequiste Libraries
------------------------------------------------------------------------------

PDAL uses the `AppVeyor`_ continuous integration platform for building and
testing itself on Windows. The configuration that PDAL uses is valuable
raw materials for configuring your own environment because the PDAL
team must keep it up to date with both the OSGeo4W environment and
the Microsoft compiler situation.

You can see the current AppVeyor configuration at
https://github.com/PDAL/PDAL/blob/master/appveyor.yml The most interesting
bits are the ``install`` section, the ``config.cmd``, and the ``build.cmd``.

The AppVeyor configuration installs OSGeo4W and all of PDAL's prerequisites
via the command line.

After downloading the `OSGeo4W setup`_, you can invoke it via the command
line to install PDAL's prerequisite packages.

::

   C:\temp\osgeo4w-setup.exe -q -k -r -A -s http://download.osgeo.org/osgeo4w/ -a x86_64 ^
         -P eigen,gdal,geos,hexer,iconv,laszip,libgeotiff,libpq,libtiff,^
            libxml2,msys,nitro,laz-perf,proj,zlib,python3-core,python3-devel,^
            python3-numpy,oci,oci-devel,laz-perf,jsoncpp -R c:/OSGeo4W64

.. note::

    The package list here might change over time. The canonnical location
    to learn the OSGeo4W prerequisite list for PDAL is the ``appveyor.yml``
    file in PDAL's source tree.

.. seealso::

    If you don't wish to run via the command line, you can choose the GUI
    for installation. Visit :ref:`workshop-osgeo4w` for a description, and then
    choose all of the listed support libraries (minus ``PDAL`` of course)
    to schedule them for installation.

.. warning::

    There are a number of package scripts that assume ``c:/OSGeo4W64`` as the
    installation path, and it is likely that you will run into some
    trouble attempting to install in other locations. It's possible it will
    work with some elbow grease, but it might not work out of the box.

.. _`AppVeyor`: https://ci.appveyor.com/project/hobu/pdal/history
.. _`OSGeo4W setup`: http://download.osgeo.org/osgeo4w/osgeo4w-setup-x86_64.exe


Fetching the Source
------------------------------------------------------------------------------

Get the source code for PDAL. Presumably you have `GitHub for Windows`_ or
something like it. Run a "git shell" and clone the repository into the
directory of your choice.

   ::

      c:\dev> git clone https://github.com/PDAL/PDAL.git

.. _`GitHub for Windows`: https://desktop.github.com/

Switch to the ``-maintenance`` branch.

   ::

      c:\dev> git checkout 1.7-maintenance


   .. note::

        PDAL's active development branch is ``master``, and you are welcome to
        build it, but is not as stable as the major-versioned release
        branches are likely to be.

Configuration
------------------------------------------------------------------------------

PDAL uses `CMake`_ for its build configuration. You will need to install CMake
and have it available on your path to configure PDAL.

Invoke your ``cmake`` command to configure the PDAL.

::

    cmake -G "NMake Makefiles" .

A fully-featured build will require more specification of libraries, enabled
features, and their locations. There are two places in the source tree
for inspiration on this topic.

1. The AppVeyor build configuration https://github.com/PDAL/PDAL/blob/master/scripts/appveyor/config.cmd#L26

2. Howard Butler's example build configuration https://github.com/PDAL/PDAL/blob/master/cmake/examples/hobu-windows.bat


.. note::

    Placing your command in a ``.bat`` file will make for easy reuse.

Building
------------------------------------------------------------------------------

If you chose ``NMake Makefiles`` as your CMake generator, you can
invoke the build by calling **nmake**:

::

    nmake /f Makefile


If you chose "Visual Studio 14 Win64" as your CMake generator, open ``PDAL.sln``
and chose your configuration to build.

Running
------------------------------------------------------------------------------

After you've built the tree, you can run ``pdal.exe`` by issuing it

::

    c:\dev\pdal\bin\pdal.exe

.. note::

    You need to have your OSGeo4W shell active to enable access to
    PDAL's dependencies. Issue ``c:\osgeo4w64\bin\o4w_env.bat`` in
    your shell to activiate it.
