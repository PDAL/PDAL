.. _download:

******************************************************************************
Download
******************************************************************************


.. contents::
   :depth: 3
   :backlinks: none


Current Release(s)
------------------------------------------------------------------------------

* **2016-12-15** `PDAL-1.4.0-src.tar.gz`_ `Release Notes`_ (`md5`_)

.. _`Release Notes`: https://github.com/PDAL/PDAL/releases/tag/1.4.0

.. _`PDAL-1.4.0-src.tar.gz`: http://download.osgeo.org/pdal/PDAL-1.4.0-src.tar.gz
.. _`md5`: http://download.osgeo.org/pdal/PDAL-1.4.0-src.tar.gz.md5
.. _`DebianGIS`: http://wiki.debian.org/DebianGis


Past Releases
------------------------------------------------------------------------------

* **2016-08-29** `PDAL-1.3.0-src.tar.gz`_ `Release Notes`_
* **2016-03-31** `PDAL-1.2.0-src.tar.gz`_ `Release Notes`_


.. _`PDAL-1.3.0-src.tar.gz`: http://download.osgeo.org/pdal/PDAL-1.3.0-src.tar.gz
.. _`PDAL-1.2.0-src.tar.gz`: http://download.osgeo.org/pdal/PDAL-1.2.0-src.tar.gz



.. _source:

Development Source
------------------------------------------------------------------------------

The main repository for PDAL is located on github at https://github.com/PDAL/PDAL

You can obtain a copy of the active source code by issuing the following command::

    git clone git@github.com:PDAL/PDAL.git pdal



Binaries
------------------------------------------------------------------------------

Docker
................................................................................

The fastest way to get going with PDAL is to use the Docker build. See the
tutorial at :ref:`docker` for more information.

::

    docker pull pdal/pdal:1.2


Windows
................................................................................

A 1.1.0 release of PDAL is available via `OSGeo4W`_. It is only 64-bit at this
time. Use the :ref:`docker` builds if you want to use the PDAL :ref:`apps`, otherwise,
a call for help with building current Windows PDAL builds is at https://lists.osgeo.org/pipermail/pdal/2016-November/001089.html

RPMs
................................................................................

RPMs for PDAL are available at https://copr.fedorainfracloud.org/coprs/neteler/pdal/

Debian
................................................................................

Debian packages are now available on `Debian Unstable`_.

.. _`OSGeo4W`: http://trac.osgeo.org/osgeo4w/
.. _`Debian Unstable`: https://tracker.debian.org/pkg/pdal
.. _`LASzip`: http://laszip.org
