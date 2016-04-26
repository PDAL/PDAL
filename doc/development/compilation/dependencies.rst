.. _dependencies:

==============================================================================
Dependencies
==============================================================================

:Author: Howard Butler
:Contact: howard@hobu.co
:Date: 11/03/2015

PDAL explicitly stands on the shoulders of giants that have come before it.
Specifically, PDAL depends on a number of libraries to do its work. Most are
not required. For optional dependencies, PDAL utilizes a dynamically-linked
plugin architecture that loads them at runtime.


Required Dependencies
------------------------------------------------------------------------------

GDAL
..............................................................................

PDAL uses GDAL for spatial reference system description manipulation, and image
reading supporting for the NITF driver, and :ref:`writers.oci` support. In
conjunction with GeoTIFF_, GDAL is used to convert GeoTIFF keys and OGC WKT SRS
description strings into formats required by specific drivers. While PDAL can
be built without GDAL support, if you want SRS manipulation and description
ability, you must have GDAL (and GeoTIFF_) linked in at compile time.

Obtain `GDAL`_ via whatever method is convenient.  Linux platforms such as
`Debian`_ have `DebianGIS`_, Mac OS X has the `KyngChaos`_ software frameworks,
and Windows has the `OSGeo4W`_ platform.

* GDAL 1.9+ is required.

.. warning::
    If you are using `OSGeo4W`_ as your provider of GDAL, you must make sure
    to use the GDAL 1.9 package.

Optional Dependencies
------------------------------------------------------------------------------

GeoTIFF
..............................................................................

PDAL uses GeoTIFF in conjunction with GDAL for GeoTIFF key support in the
LAS driver.  Obtain `GeoTIFF`_ from the same place you got `GDAL`_.

* libgeotiff 1.3.0+ is required

.. note::
    `GDAL` surreptitiously embeds a copy of `GeoTIFF`_ in its library build
    but there is no way for you to know this.  In addition to embedding
    libgeotiff, it also strips away the library symbols that PDAL needs,
    meaning that PDAL can't simply link against `GDAL`_.  If you are
    building both of these libraries yourself, make sure you build GDAL
    using the "External libgeotiff" option, which will prevent the
    insanity that can ensue on some platforms.  `OSGeo4W`_ users, including
    those using that platform to link and build PDAL themselves, do
    not need to worry about this issue.

Proj.4
..............................................................................

Proj.4_ is the projection engine that PDAL uses for the
:ref:`filters.reprojection` filter. It is used by GDAL.

.. note::

    Proj.4 4.9.0+ is required if you need vertical datum
    transformation support. Otherwise, older versions should be
    sufficient.

libxml2
..............................................................................

libxml2_ is used to serialize PDAL dimension descriptions into XML for the
database drivers such as :ref:`writers.oci`, :ref:`readers.sqlite`, or
:ref:`readers.pgpointcloud`

.. note::

    libxml 2.7.0+ is required. Older versions may also work but are untested.

`OCI`_
..............................................................................

Obtain the `Oracle Instant Client`_ and install in a location on your system.
Be sure to install both the "Basic" and the "SDK" modules. Set your
``ORACLE_HOME`` environment variable system- or user-wide to point to this
location so the CMake configuration can find your install. OCI is used by
both :ref:`writers.oci` and :ref:`readers.oci` for Oracle
Point Cloud read/write support.

.. warning::
    `OCI`_'s libraries are inconsistently named.  You may need to create
    symbolic links for some library names in order for the `CMake`_ to find
    them::

        cd $ORACLE_HOME
        ln -s libocci.so.11.1 libocci.so
        ln -s libclntsh.so.11.1 libclntsh.so
        ln -s libociei.so.11.1 libociei.so

* OCI 10g+ is required.

.. note::
    MSVC should only require the oci.lib and oci.dll library and dlls.

Points2Grid
..............................................................................

`Points2Grid`_ is a library with a simple `CMake`-based build system that
provides simple, out-of-process interpolation of large point sets using
Boost_. It can be obtained via github.com at https://github.com/CRREL/points2grid
It is used by :ref:`writers.p2g` to output point cloud interpolation.

Hexer
..............................................................................

`Hexer`_ is a library with a simple `CMake`-based build system that
provides simple hexagon gridding of large point sets for density surface
generation and boundary approximation. It can be obtained via github.com at
https://github.com/hobu/hexer It is used by :ref:`filters.hexbin` to output
density surfaces and boundary approximations.

Nitro
..............................................................................

Nitro is a library that provides `NITF`_ support for PDAL to write LAS-in-NITF
files for :ref:`writers.nitf`. PDAL can only use a fork of Nitro located at
http://github.com/hobu/nitro instead of the mainline tree for two reasons:

1) The fork contains a simple `CMake`-based build system
2) The fork properly dynamically links on Windows to maintain LGPL compliance.

It is expected that the fork will go away once these items are incorporated into
the main source tree.


LASzip
..............................................................................

`LASzip`_ is a library with a simple `CMake`-based build system that
provides periodic compression of `ASPRS LAS`_ data. It is used by the
:ref:`writers.las` and :ref:`readers.las` to provide
compressed LAS support.

laz-perf
..............................................................................

In addition to `LASzip`_, you can use the alternative `laz-perf`_ library.
`laz-perf`_ provides slightly faster decompression capability for typical
LAS files. It is also used as a compression type for :ref:`writers.oci` and
:ref:`writers.sqlite`

.. _`laz-perf`: https://github.com/verma/laz-perf/

PCL
..............................................................................

The `Point Cloud Library (PCL)`_ is used by the :ref:`ground_command`,
:ref:`pcl_command`, :ref:`writers.pcd`, :ref:`readers.pcd`, and
:ref:`filters.pclblock` to provide support for various PCL-related operations.

PCL must be 1.7.2+. We do our best to keep this up-to-date with PCL master.

.. note::
    `Homebrew`_-based OSX builds use PCL 1.7.2, but you may need to switch
    of `VTK`_ support depending on the configuration.

.. _`Homebrew`: http://brew.sh
.. _`VTK`: http://vtk.org

.. _`ASPRS LAS`: http://www.asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html
.. _`LASzip`: http://laszip.org
.. _`NITF`: http://en.wikipedia.org/wiki/National_Imagery_Transmission_Format
.. _`Nitro`: http://nitro-nitf.sourceforge.net/wikka.php?wakka=HomePage

.. _`Oracle Instant Client`: http://www.oracle.com/technology/tech/oci/instantclient/index.html
.. _`OCI`: http://www.oracle.com/technology/tech/oci/index.html
.. _`Oracle Point Cloud`: http://download.oracle.com/docs/cd/B28359_01/appdev.111/b28400/sdo_pc_pkg_ref.htm
.. _`DebianGIS`: http://wiki.debian.org/DebianGis
.. _`Debian`: http://www.debian.org
.. _`KyngChaos`: http://www.kyngchaos.com/software/unixport
.. _`OSGeo4W`: http://trac.osgeo.org/osgeo4w/

.. _Boost: http://www.boost.org
.. _GDAL: http://www.gdal.org
.. _Proj.4: http://trac.osgeo.org/proj
.. _GeoTIFF: http://trac.osgeo.org/geotiff
.. _libxml2: http://xmlsoft.org
.. _CMake: http://www.cmake.org
.. _`libpq`: http://www.postgresql.org/docs/9.3/static/libpq.html

.. _`Points2Grid`: https://github.com/CRREL/points2grid
.. _`Point Cloud Library (PCL)`: http://pointclouds.org

