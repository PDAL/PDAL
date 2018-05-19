.. _download:

******************************************************************************
Download
******************************************************************************


.. contents::
   :depth: 3
   :backlinks: none


Current Release(s)
------------------------------------------------------------------------------

* **2018-05-13** `PDAL-1.7.2-src.tar.gz`_ `Release Notes`_ (`md5`_)

.. _`PDAL-1.7.2-src.tar.gz`: http://download.osgeo.org/pdal/PDAL-1.7.2-src.tar.gz
.. _`Release Notes`: https://github.com/PDAL/PDAL/releases/tag/1.7.2
.. _`md5`: http://download.osgeo.org/pdal/PDAL-1.7.2-src.tar.gz.md5


Past Releases
------------------------------------------------------------------------------

* **2018-04-06** `PDAL-1.7.1-src.tar.gz`_
* **2018-04-05** `PDAL-1.7.0-src.tar.gz`_
* **2017-10-12** `PDAL-1.6.0-src.tar.gz`_


.. _`PDAL-1.7.1-src.tar.gz`: http://download.osgeo.org/pdal/PDAL-1.7.1-src.tar.gz
.. _`PDAL-1.7.0-src.tar.gz`: http://download.osgeo.org/pdal/PDAL-1.7.0-src.tar.gz
.. _`PDAL-1.6.0-src.tar.gz`: http://download.osgeo.org/pdal/PDAL-1.6.0-src.tar.gz


.. _source:

Development Source
------------------------------------------------------------------------------

The main repository for PDAL is located on github at
https://github.com/PDAL/PDAL.

You can obtain a copy of the active source code by issuing the following
command

::

    git clone https://github.com/PDAL/PDAL.git pdal


Binaries
------------------------------------------------------------------------------

In this section we list a number of the binary distributions of PDAL. The table
below is intended to provide an overview of some of the differences between the
various distributions, as not all features can be enabled in every
distribution. This table only summarizes the differences between distributions,
and there are several plugins that are not built for any of the distributions.
These include Delaunay, GeoWave, MATLAB, MBIO, MRSID, OpenSceneGraph, RDBLIB,
and RiVLib. To enable any of these plugins, the reader will need to install any
required dependencies and build PDAL from source.

.. csv-table:: PDAL Distribution Feature Comparison
   :header: "", "Docker", "OSGeo4W", "RPMs", "Debian", "Alpine", "Conda"
   :widths: 20, 20, 20, 20, 20, 20, 20

   "Platform(s)", "linux", "win", "linux", "linux", "linux", "win, mac, linux"
   "CPD", "X", "", "", "", "X", ""
   "Greyhound", "X", "X", "", "X", "X", "X"
   "Hexbin", "X", "X", "X", "", "X", "X"
   "Icebridge", "X", "", "X", "X", "X", "X"
   "laszip", "X", "X", "X", "", "X", "X"
   "laz-perf", "X", "X", "", "", "X", "X"
   "NITF", "X", "X", "", "", "X", "X"
   "OCI", "", "X", "", "", "", ""
   "PCL", "", "", "X", "", "", "X"
   "pgpointcloud", "X", "X", "X", "X", "X", "X"
   "Python", "X", "X", "", "X", "X", "X"
   "SQLite", "X", "X", "", "X", "X", "X"


Docker
................................................................................

The fastest way to get going with PDAL is to use the Docker build. See the
:ref:`Docker tutorial <docker>` for more information.

::

    docker pull pdal/pdal:1.7


Windows
................................................................................

Windows builds are available via `OSGeo4W`_ (64-bit only). See the
:ref:`workshop-osgeo4w` page for more detailed information.

.. _`OSGeo4W`: http://trac.osgeo.org/osgeo4w/


RPMs
................................................................................

RPMs for PDAL are available at
https://copr.fedorainfracloud.org/coprs/neteler/pdal/.


Debian
................................................................................

Debian packages are now available on `Debian Unstable`_.

.. _`Debian Unstable`: https://tracker.debian.org/pkg/pdal


Alpine
................................................................................

`Alpine`_ is a linux distribution that is compact and frequently used with
Docker images. Alpine packages for PDAL are available at
https://pkgs.alpinelinux.org/packages?name=*pdal*&branch=edge.

Users have a choice of three separate packages.

1. ``pdal`` will install the PDAL binaries only, and is suitable for users who
will be using the PDAL command line applications.

2. ``pdal-dev`` will install development files which are required for users
building their own software that will link against PDAL.

3. ``py-pdal`` will install the PDAL Python extension.

Note that all of these packages reside in Alpine's ``edge/testing`` repository,
which must be added to your Alpine repositories list. Information on adding and
updating repositories can be found in the Alpine documentation.

To install one or more packages on Alpine, use the following command.

::

    apk add [package...]

For example, the following command will install both the PDAL application and
the Python extension.

::

    apk add py-pdal pdal

.. _`Alpine Linux`: https://www.alpinelinux.org/


Conda
................................................................................

`Conda`_ can be used on multiple platforms (Windows, macOS, and Linux) to
install software packages and manage environments. Conda packages for PDAL are
available at https://anaconda.org/conda-forge/pdal.

Conda installation instructions can be found on the Conda website. The
instructions below assuming you have a working Conda installation on your
system.

Users have a choice of two separate packages.

1. ``pdal`` will install the PDAL binaries **and** development files.

2. ``python-pdal`` will install the PDAL Python extension.

To install one or more Conda packages, use the following command.

::

    conda install [-c channel] [package...]

Because the PDAL package (and it's dependencies) live in the `conda-forge`_
channel, the command to install both the PDAL application and the Python
extension is

::

    conda install -c conda-forge pdal python-pdal

It is strongly recommended that you make use of Conda's environment management
system and install PDAL in a separate environment (i.e., not the base
environment). Instructions can be found on the Conda website.

.. _`Conda`: https://conda.io/docs/
.. _`conda-forge`: https://conda-forge.org/
