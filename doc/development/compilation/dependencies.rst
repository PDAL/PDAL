.. _dependencies:

==============================================================================
Dependencies
==============================================================================

PDAL depends on a number of libraries to do its work.  You should make sure
those dependencies are installed on your system before installing PDAL
or use a packaging system that will automatically ensure that prerequisites
are satisified.  Packaging system such as `apt`_ or `Conda`_ can
be used to install dependencies on your system.

.. _`apt`: https://help.ubuntu.com/lts/serverguide/apt.html
.. _`Conda`: https://conda.io/en/latest/

Required Dependencies
------------------------------------------------------------------------------

GDAL (2.2+)
..............................................................................

PDAL uses GDAL for spatial reference system description manipulation, and image
reading supporting for the NITF driver, and :ref:`writers.oci` support. In
conjunction with GeoTIFF_, GDAL is used to convert GeoTIFF keys and OGC WKT SRS
<<<<<<< HEAD
description strings into formats required by specific drivers.  ::
=======
description strings into formats required by specific drivers. While PDAL can
be built without GDAL support, if you want SRS manipulation and description
ability, you must have GDAL (and GeoTIFF_) linked in at compile time.

Obtain `GDAL`_ via whatever method is convenient.  Linux platforms such as
`Debian`_ have `DebianGIS`_, Mac OS X has the `KyngChaos`_ software frameworks,
and Windows has the ``_ platform.

* GDAL 1.9+ is required.
>>>>>>> f881aa2d15d4d177f3fd8159ea72751a183dd7ec

    Source: https://github.com/OSGeo/gdal
    Conda: https://anaconda.org/conda-forge/gdal

GeoTIFF (1.3+)
..............................................................................

PDAL uses GeoTIFF in conjunction with GDAL for GeoTIFF key support in the
LAS driver.  GeoTIFF is typically a dependency of GDAL, so installing GDAL
from a package will generally install GeoTIFF as well. ::

    Source: https://github.com/OSGeo/libgeotiff
    Conda: https://anaconda.org/conda-forge/geotiff

.. note::
    `GDAL` surreptitiously embeds a copy of `GeoTIFF`_ in its library build
    but there is no way for you to know this.  In addition to embedding
    libgeotiff, it also strips away the library symbols that PDAL needs,
    meaning that PDAL can't simply link against `GDAL`_.  If you are
    building both of these libraries yourself, make sure you build GDAL
    using the "External libgeotiff" option, which will prevent the
    insanity that can ensue on some platforms.  `Conda Forge`_ users, including
    those using that platform to link and build PDAL themselves, do
    not need to worry about this issue.

Optional Dependencies
------------------------------------------------------------------------------

LASzip (Latest package/source recommended)
..............................................................................

`LASzip`_ is a library with a `CMake`-based build system that
provides periodic compression of `ASPRS LAS`_ data. It is used by the
:ref:`writers.las` and :ref:`readers.las` to provide
compressed LAS support.::

    Source: https://github.com/LASzip/LASzip
    Conda: https://anaconda.org/conda-forge/laszip

laz-perf (Latest package/source recommended)
..............................................................................

laz-perf provides an alternative LAS compression/decompression engine that
may be slightly faster in some circumstances.  laz-perf supports fewer LAS
point types and versions than does LASzip.  It is also used as a
compression type for :ref:`writers.oci` and :ref:`writers.sqlite`::

    Source: https://github.com/verma/laz-perf/
    Conda: https://anaconda.org/conda-forge/laz-perf

libxml2  (2.7+)
..............................................................................

libxml2_ is used to serialize PDAL dimension descriptions into XML for the
database drivers such as :ref:`writers.oci`, :ref:`readers.sqlite`, or
:ref:`readers.pgpointcloud`.::

    Source: http://www.xmlsoft.org/
    Conda: https://anaconda.org/conda-forge/libxml2

Plugin Dependencies
------------------------------------------------------------------------------

PDAL comes with optional plugin stages that require other libraries in order
to run.  Many of these libraries are licensed in a way incompatible with
the PDAL license or they may be commercial products that require purchase.

OCI (10g+)
..............................................................................

Obtain the `Oracle Instant Client`_ and install in a location on your system.
Be sure to install both the "Basic" and the "SDK" modules. Set your
``ORACLE_HOME`` environment variable system- or user-wide to point to this
location so the CMake configuration can find your install. OCI is used by
both :ref:`writers.oci` and :ref:`readers.oci` for Oracle
Point Cloud read/write support.  In order to obtain the OCI libraries
you must register with Oracle.::

    Libraries: https://www.oracle.com/technetwork/database/database-technologies/instant-client/downloads/index.html

Nitro (Requires specific source package)
..............................................................................

Nitro is a library that provides `NITF`_ support for PDAL to write LAS-in-NITF
files for :ref:`writers.nitf`.  You must use the specific version of Nitro
referenced below for licensing and compatibility reasons.::

    Source: http://github.com/hobu/nitro

PCL  (1.7.2+)
..............................................................................

The `Point Cloud Library (PCL)`_ is used by the :ref:`pcl_command`,
:ref:`writers.pcd`, :ref:`readers.pcd`, and :ref:`filters.pclblock` to provide
support for various PCL-related operations.::

    Source: https://github.com/PointCloudLibrary/pcl
    Conda: https://anaconda.org/conda-forge/pcl

TileDB  (1.4.1+)
..............................................................................

`TileDB`_ is an efficient multi-dimensional array management system which
introduces a novel on-disk format that can effectively store dense and sparse
array data with support for fast updates and reads. It features excellent
compression, and an efficient parallel I/O system with high scalability. It is
used by :ref:`writers.tiledb` and :ref:`readers.tiledb`.::

    Source: https://github.com/TileDB-Inc/TileDB
    Conda: https://anaconda.org/conda-forge/tiledb

.. _`ASPRS LAS`: http://www.asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html
.. _`LASzip`: http://laszip.org
.. _`NITF`: http://en.wikipedia.org/wiki/National_Imagery_Transmission_Format
.. _`Nitro`: http://nitro-nitf.sourceforge.net/wikka.php?wakka=HomePage

.. _`Oracle Instant Client`: http://www.oracle.com/technology/tech/oci/instantclient/index.html
.. _`OCI`: http://www.oracle.com/technology/tech/oci/index.html
.. _`Oracle Point Cloud`: http://download.oracle.com/docs/cd/B28359_01/appdev.111/b28400/sdo_pc_pkg_ref.htm
.. _`DebianGIS`: http://wiki.debian.org/DebianGis
.. _`Debian`: http://www.debian.org
.. _`Conda Forge`: https://anaconda.org/conda-forge/pdal

.. _GDAL: http://www.gdal.org
.. _GeoTIFF: http://trac.osgeo.org/geotiff
.. _libxml2: http://xmlsoft.org
.. _CMake: http://www.cmake.org
.. _`Point Cloud Library (PCL)`: http://pointclouds.org
.. _`TileDB`: https://www.tiledb.io
