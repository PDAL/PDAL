.. _faq:

******************************************************************************
FAQ
******************************************************************************

.. index:: pronounce

* How do you pronounce PDAL?

  The proper spelling of the project name is PDAL, in uppercase. It is
  pronounced to rhyme with "GDAL".

* Why do I get the error "Couldn't create ... stage of type ..."?

  In almost all cases this error occurs because you're trying to run a stage
  that is built as a plugin and the plugin (a shared library file or DLL)
  can't be found by the pdal.  You can verify whether the plugin can
  be found by pdal by running "pdal --drivers"

  If you've built pdal yourself, make sure you've requested to build the
  plugin in question (set BUILD_PLUGIN_PCL=ON, for example, in CMakeCache.txt).

  If you've successfully built the plugin, you should have created a
  shared object called
  libpdal_plugin_<plugin type>_<plugin name>.<shared library extension> that
  should be installed in a location where pdal can find it.  pdal will search
  the following paths for plugins: ".", "./lib", "../lib", "./bin", "../bin".

  You can also override the default search path by setting the environment
  variable PDAL_DRIVER_PATH to a list of directories that pdal should search
  for plugins.

* What is PDAL's relationship to PCL?

  PDAL is PCL's data translation cousin. PDAL is focused on providing a
  declarative pipeline syntax for orchestrating translation operations.
  PDAL can also use PCL through the :ref:`filters.pclblock` mechanism.

  .. seealso::

        :ref:`about_pcl` describes PDAL and PCL's relationship.

* What is PDAL's relationship to libLAS?

  The idea behind libLAS was limited to LIDAR data and basic
  manipulation. libLAS was also trying to be partially compatible
  with LASlib and LAStools. PDAL, on the other hand, aims to be
  a ultimate library and a set of tools for manipulating and processing
  point clouds and is easily extensible by its users.

* Are there any command line tools in PDAL similar to LAStools?

  Yes. The ``pdal`` command provides a wide range of features which go
  far beyond basic LIDAR data processing. Additionally, PDAL is licensed
  under an open source license (this applies to the whole library and
  all command line tools).

  .. seealso::

        :ref:`apps` describes application operations you can
        achieve with PDAL.

* Is there any compatibility with libLAS's LAS Utility Applications or LAStools?

  No. The the command line interface was developed from scratch with
  focus on usability and readability. You will find that the ``pdal``
  command has several well-organized subcommands such as ``info``
  or ``translate`` (see :ref:`apps`).
