.. _docker:

******************************************************************************
Docker
******************************************************************************

:Author: Howard Butler
:Contact: howard@hobu.co
:Date: 12/02/2015

.. index:: Docker

Introduction
------------------------------------------------------------------------------

It's a giant pain to build everything yourself. Like the :ref:`vagrant` configuration,
a PDAL build based on `Docker`_ is also available. This document describes how
to use it to operate on data.


.. seealso::
    The `What is Docker <https://www.docker.com/what-docker>`__ document describes
    in more detail what exactly Docker is. Think of it as a virtualization platform
    that doesn't have to be "built" every time from scratch like :ref:`vagrant`.

Prerequisites
------------------------------------------------------------------------------

Install `Docker`_.

* `Windows <http://docs.docker.com/windows/started/>`__
* `OSX <http://docs.docker.com/mac/started/>`__
* `Linux <http://docs.docker.com/linux/started/>`__

.. note::

    This tutorial will assume you are running on Windows, but the same commands should
    work in OSX or Linux too.

Data
------------------------------------------------------------------------------

Copy some LAS data to the directory you're running the shell


What you get
------------------------------------------------------------------------------

The :ref:`docker`_ configuration that PDAL provides contains nearly
every possible feature except for `Oracle Point Cloud`_ support. Things it
includes are:

* :ref:`filters.hexbin`
* :ref:`filters.reprojection`
* :ref:`readers.nitf` and :ref:`writers.nitf`
* `LASzip`_ support in :ref:`readers.las` and :ref:`writers.las`
* Interpolation with `Points2Grid`_ using :ref:`writers.p2g`
* Python manipulation using :ref:`filters.predicate` and :ref:`filters.programmable`
* `PCL`_ support via :ref:`filters.pclblock`, :ref:`readers.pcd`,
  :ref:`writers.pcd`, :ref:`ground_command`, and :ref:`pcl_command`
* :ref:`writers.geowave`

Head to :ref:`pipeline` for more information on using PDAL pipelines. Two pipelines
are provided in ``/home/vagrant`` that are used to load the ``st-helens-small.las``
file into `pgpointcloud`_.

.. _`Points2Grid`: https://github.com/CRREL/points2grid
.. _`Oracle Point Cloud`: http://docs.oracle.com/cd/B28359_01/appdev.111/b28400/sdo_pc_pkg_ref.htm
.. _`pgpointcloud`: https://github.com/pramsey/pointcloud

.. _`LASzip`: http://laszip.org
.. _`VirtualBox`: https://www.virtualbox.org/
.. _`GDAL`: http://gdal.org
.. _`MapServer`: http://mapserver.org
.. _`Mapnik`: http://mapnik.org
.. _`PCL`: http://www.pointclouds.org
