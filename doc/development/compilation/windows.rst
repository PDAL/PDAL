.. _building_windows:

==============================================================================
Building Under Windows
==============================================================================

:Author: Howard Butler
:Contact: howard at hobu.co
:Date: 03/20/2019

.. note::

    :ref:`conda` contains a pre-built up-to-date 64 bit Windows binary. It
    is fully-featured, and if you do not need anything custom, it is likely
    the fastest way to get going.


Introduction
------------------------------------------------------------------------------

Pre-built binary packages for Windows are available via :ref:`conda` (64-bit version),
and all of the prerequisites required for compilation of a fully featured build
are also available via that packaging system. This document assumes you
will be using Conda Forge as your base, and anything more advanced is beyond
the scope of the document.

.. note::

    The AppVeyor build system uses the PDAL project's configuration on the Conda Forge
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

Prerequisite Libraries
------------------------------------------------------------------------------

PDAL uses the `AppVeyor`_ continuous integration platform for building and
testing itself on Windows. The configuration that PDAL uses is valuable
raw materials for configuring your own environment because the PDAL
team must keep it up to date with both the :ref:`conda` environment and
the Microsoft compiler situation.

You can see the current AppVeyor configuration at
https://github.com/PDAL/PDAL/blob/master/appveyor.yml The most interesting bits
are the ``install`` section, the ``config.cmd``, and the ``build.cmd`` scripts.
The AppVeyor configuration already has Miniconda installed, and the
``config.cmd`` script installs all of PDAL's prerequisites via the command
line.


::

   conda install geotiff laszip nitro curl ^
      gdal pcl cmake eigen ninja libgdal ^
      zstd numpy xz libxml2 laz-perf qhull ^
      sqlite hdf5 tiledb conda-build ninja -y

.. note::

    The package list here might change over time. The canonnical location
    to learn the  prerequisite list for PDAL is the ``scripts/appveyor/test/build.cmd``
    file in PDAL's source tree.

.. _`AppVeyor`: https://ci.appveyor.com/project/hobu/pdal/history


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

      c:\dev> git checkout 1.9-maintenance


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

2. Howard Butler's example build configuration https://github.com/PDAL/PDAL/blob/master/scripts/conda/wind64.bat


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

    You may need to have your Conda environment active to enable access to
    PDAL's dependencies.
