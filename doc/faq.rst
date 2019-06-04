.. _faq:

******************************************************************************
FAQ
******************************************************************************

.. index:: pronounce

* How do you pronounce PDAL?

  The proper spelling of the project name is PDAL, in uppercase. It is
  pronounced to rhyme with "GDAL".

  .. it is properly pronounced like the dog though :) -- hobu

|

* Why do I get the error "Couldn't create ... stage of type ..."?

  In almost all cases this error occurs because you're trying to run a stage
  that is built as a plugin and the plugin (a shared library file or DLL)
  can't be found by pdal.  You can verify whether the plugin can
  be found by running ``pdal --drivers``

  If you've built pdal yourself, make sure you've requested to build the
  plugin in question (set BUILD_PLUGIN_PCL=ON, for example, in CMakeCache.txt).

  If you've successfully built the plugin, a
  shared object called

  ::

    libpdal_plugin_<plugin type>_<plugin name>.<shared library extension>

  should have been created that's installed in a location where pdal
  can find it.  pdal will search
  the following paths for plugins: ``.``, ``./lib``, ``../lib``, ``./bin``,
  ``../bin``.

  You can also override the default search path by setting the environment
  variable ``PDAL_DRIVER_PATH`` to a list of directories that pdal should search
  for plugins.

.. index:: PCL

* Why am I using 100GB of memory when trying to process a 10GB LAZ file?

  If you're performing an operation that is using
  :ref:`standard mode <processing_modes>`, PDAL will read all points into
  memory at once.  Compressed files, like LAZ, can decompress to much larger
  sizes before PDAL can process the data. Furthermore, some operations
  (notably :ref:`DEM creation<writers.gdal>`) can use large amounts of
  additional memory during processing before the output can be written.
  Depending on the operation, PDAL will attempt operate in
  :ref:`stream mode <processing_modes>` to
  limit memory consumption when possible.

|

* What is PDAL's relationship to PCL?

  PDAL is PCL's data translation cousin. PDAL is focused on providing a
  declarative pipeline syntax for orchestrating translation operations.
  PDAL can also use PCL through the :ref:`filters.pclblock` mechanism.
  PDAL also supports reading and writing PCL PCD files using :ref:`readers.pcd`
  and :ref:`writers.pcd`.

  .. seealso::

        :ref:`about_pcl` describes PDAL and PCL's relationship.

* What is PDAL's relationship to libLAS?

  The idea behind libLAS was limited to LIDAR data and basic
  manipulation. libLAS was also trying to be partially compatible
  with LASlib and LAStools. PDAL, on the other hand, aims to be
  a ultimate library and a set of tools for manipulating and processing
  point clouds and is easily extensible by its users. Howard Butler
  talked more about this history in a `GeoHipster interview`_ in
  2018.

.. _`GeoHipster interview`: http://geohipster.com/2018/03/05/howard-butler-like-good-song-open-source-software-chance-immortal/

|

* Are there any command line tools in PDAL similar to LAStools?

  Yes. The :ref:`pdal <apps>` command provides a wide range of features which go
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

* I get GeoTIFF errors. What can I do about them?

  ::

    (readers.las Error) Geotiff directory contains key 0 with short entry and more than one value.

  If :ref:`readers.las` is outputting error messages about GeoTIFF, this means
  the keys that were written into your file were incorrect or at least not
  readable by `libgeotiff`_. Rewrite the file using PDAL to fix the issue:

  ::

    pdal translate badfile.las goodfile.las --writers.las.forward=all

.. _`libgeotiff`: https://trac.osgeo.org/geotif
