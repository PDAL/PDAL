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

The entire website is available as a single PDF at http://pdal.io/PDAL.pdf

Download
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2

   download

Community
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2

   community

Usage
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2
   :glob:

   apps/index
   quickstart
   stages/readers
   stages/writers
   stages/filters
   pipeline
   tutorial/index

Workshop
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2

   Workshop <workshop/index>

Development
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2

   development/index
   api/index
   faq
   copyright





Indices and tables
--------------------------------------------------------------------------------

* :ref:`genindex`
* :ref:`search`

.. _`GDAL`: http://www.gdal.org
.. _`BSD`: http://www.opensource.org/licenses/bsd-license.php
.. _`point cloud data`: http://en.wikipedia.org/wiki/Point_cloud
.. _`LIDAR`: http://en.wikipedia.org/wiki/LIDAR

