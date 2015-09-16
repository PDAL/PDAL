.. _faq:

******************************************************************************
FAQ
******************************************************************************

* How do you pronounce PDAL?

  The proper spelling of the project name is PDAL, in uppercase. It is
  pronounced to rhyme with "GDAL".

* What is PDAL's relationship to PCL?

  PDAL is PCL's data translation cousin. PDAL is focused on providing a
  declarative pipeline syntax for orchestrating translation operations.
  PDAL can also use PCL through the :ref:`filters.pclblock` mechanism.

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

* Is there any compatibility with libLAS's LAS Utility Applications or LAStools?

  No. The the command line interface was developed from scratch with
  focus on usability and readability. You will find that the ``pdal``
  command has several well-organized subcommands such as ``info``
  or ``translate`` (see :ref:`apps`).
