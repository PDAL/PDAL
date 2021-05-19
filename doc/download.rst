.. _download:

******************************************************************************
Download
******************************************************************************


.. contents::
   :depth: 3
   :backlinks: none


Current Release(s)
------------------------------------------------------------------------------

* **2020-03-20** `PDAL-2.1.0-src.tar.gz`_ `Release Notes`_ (`md5`_)

.. _`Release Notes`: https://github.com/PDAL/PDAL/releases/tag/2.1.0
.. _`md5`: https://github.com/PDAL/PDAL/releases/download/2.1.0/PDAL-2.1.0-src.tar.gz.md5


Past Releases
------------------------------------------------------------------------------

* **2019-08-23** `PDAL-2.0.1-src.tar.gz`_
* **2019-05-09** `PDAL-1.9.1-src.tar.gz`_
* **2019-04-09** `PDAL-1.9.0-src.tar.gz`_
* **2018-10-12** `PDAL-1.8.0-src.tar.gz`_
* **2018-05-13** `PDAL-1.7.2-src.tar.gz`_
* **2018-04-06** `PDAL-1.7.1-src.tar.gz`_

.. _`PDAL-2.1.0-src.tar.gz`: https://github.com/PDAL/PDAL/releases/download/2.1.0/PDAL-2.1.0-src.tar.gz
.. _`PDAL-2.0.1-src.tar.gz`: https://github.com/PDAL/PDAL/releases/download/2.0.1/PDAL-2.0.1-src.tar.gz
.. _`PDAL-1.9.1-src.tar.gz`: https://github.com/PDAL/PDAL/releases/download/1.9.1/PDAL-1.9.1-src.tar.gz
.. _`PDAL-1.9.0-src.tar.gz`: https://github.com/PDAL/PDAL/releases/download/1.9.0/PDAL-1.9.0-src.tar.gz
.. _`PDAL-1.8.0-src.tar.gz`: http://download.osgeo.org/pdal/PDAL-1.8.0-src.tar.gz
.. _`PDAL-1.7.2-src.tar.gz`: http://download.osgeo.org/pdal/PDAL-1.7.2-src.tar.gz
.. _`PDAL-1.7.1-src.tar.gz`: http://download.osgeo.org/pdal/PDAL-1.7.1-src.tar.gz


.. _source:

Development Source
------------------------------------------------------------------------------

The main repository for PDAL is located on github at
https://github.com/PDAL/PDAL.

You can obtain a copy of the active source code by issuing the following
command

::

    git clone https://github.com/PDAL/PDAL.git


Binaries
------------------------------------------------------------------------------

In this section we list a number of the binary distributions of PDAL. The table
below is intended to provide an overview of some of the differences between the
various distributions, as not all features can be enabled in every
distribution. This table only summarizes the differences between distributions,
and there are several plugins that are not built for any of the distributions.
These include Delaunay, MATLAB, MBIO, MRSID, OpenSceneGraph, RDBLIB,
and RiVLib. To enable any of these plugins, the reader will need to install any
required dependencies and build PDAL from source.

.. csv-table:: PDAL Distribution Feature Comparison
   :header: "", "Docker", "RPMs", "Debian", "Alpine", ":ref:`Conda`"
   :widths: 20, 20, 20, 20, 20, 20

   "Platform(s)", "linux", "linux", "linux", "linux", "win64, mac, linux"
   "PDAL version", "2.3", "", "", "2.2", "2.2"
   "CPD", "", "", "", "X", ""
   "E57", "X", "", "", "", "X"
   "HDF", "X", "", "", "", "X"
   "I3S", "", "", "", "", "X"
   "Icebridge", "X",  "", "", "X", "X"
   "NITF", "X",  "", "", "", "X (except Windows)"
   "pgpointcloud", "X",  "", "", "X", "X"
   "SLPK", "", "", "", "", "X"
   "TileDB", "X", "", "", "", "X (except Windows)"


Windows
................................................................................

Windows builds are available via `Conda Forge`_ (64-bit only). See the
:ref:`conda` for more detailed information.



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

Note that the PDAL package now resides in Alpine's ``edge/community`` repository,
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

.. _`Conda Forge`: https://anaconda.org/conda-forge/pdal

.. _conda:

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

Because the PDAL package (and it's dependencies) live in the `Conda Forge`_
channel, the command to install both the PDAL application and the Python
extension is

::

    conda install -c conda-forge pdal python-pdal gdal

It is strongly recommended that you make use of Conda's environment management
system and install PDAL in a separate environment (i.e., not the base
environment). Instructions can be found on the Conda website.

