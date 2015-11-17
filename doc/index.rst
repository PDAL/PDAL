.. _home:

******************************************************************************
PDAL - Point Data Abstraction Library
******************************************************************************

.. image:: ./_static/pdal_logo.png
   :alt: PDAL logo
   :align: right

PDAL is a C++ `BSD`_ library for translating and manipulating `point cloud data`_.
It is very much like the `GDAL`_ library which handles raster and vector data.
See :ref:`readers` and :ref:`writers` for data formats PDAL supports, and see
:ref:`filters` for filtering operations that you can apply with PDAL.

In addition to the library code, PDAL provides a suite of command-line
applications that users can conveniently use to process, filter, translate, and
query point cloud data.  See :ref:`apps` for more information.


Documentation
=================

.. toctree::
   :maxdepth: 2

   download
   compilation/index
   apps
   community
   stages/index
   tutorial/index
   vagrant
   pipeline
   faq
   development/index
   api/index
   metadata
   copyright


Developers and Sponsorship
------------------------------------------------------------------------------

PDAL is developed by `Howard Butler`_, `Michael Gerlek`_, `Andrew Bell`_,
`Brad Chambers`_ and `others`_.

PDAL's development is supported in coordination with efforts by the `U.S. Army
Cold Regions Research and Engineering Laboratory`_.


.. note::

    PDAL should not be confused with `PCL`_ (Point Cloud Library).  PCL is a
    library specifically designed to provide algorithmic analysis and
    modification of point clouds.  PDAL provides a limited interface to the
    facilities of PCL, but does not in general attempt to duplicate its
    capabilites.  PDAL is focused more on data access and translation than
    PCL. See :ref:`filters.pclblock` for more background.




Indices and tables
==================

* :ref:`genindex`
* :ref:`search`

.. _`PCL`: http://pointclouds.org
.. _`GDAL`: http://www.gdal.org
.. _`BSD`: http://www.opensource.org/licenses/bsd-license.php
.. _`point cloud data`: http://en.wikipedia.org/wiki/Point_cloud
.. _`LIDAR`: http://en.wikipedia.org/wiki/LIDAR
.. _`U.S. Army Cold Regions Research and Engineering Laboratory` : http://www.crrel.usace.army.mil/
.. _`Howard Butler`: http://github.com/hobu
.. _`Andrew Bell`: http://github.com/abellgithub
.. _`Michael Gerlek`: http://github.com/mpgerlek
.. _`Brad Chambers`: http://github.com/chambbj
.. _`others`: http://github.com/PDAL/PDAL/graphs/contributors



