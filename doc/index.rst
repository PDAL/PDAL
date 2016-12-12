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

News
--------------------------------------------------------------------------------

**08-29-2016**
................................................................................

PDAL 1.3.0 has been released. Visit :ref:`download` to obtain a copy of the
source code, or follow the :ref:`quickstart` to get going in a hurry with
`Docker`_.

.. _`Docker`: https://www.docker.com/

.. _`Howard Butler`: http://github.com/hobu
.. _`Brad Chambers`: http://github.com/chambbj
.. _`FOSS4GNA 2016`: https://2016.foss4g-na.org
.. _`Point cloud web services with Greyhound, Entwine, and PDAL`: https://2016.foss4g-na.org/session/point-cloud-web-services-greyhound-entwine-and-pdal
.. _`Filtering point clouds with PDAL and PCL`: https://2016.foss4g-na.org/session/filtering-point-clouds-pdal-and-pcl


Download
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2

   download

Quickstart
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2

   quickstart

Applications
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2

   apps/index

Community
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2

   community

Drivers
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2
   :glob:

   pipeline
   stages/readers
   stages/writers
   stages/filters
   dimensions

Tutorials
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2
   :glob:

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

.. toctree::
   :includehidden:

   references





Indices and tables
--------------------------------------------------------------------------------

* :ref:`genindex`
* :ref:`search`

.. _`GDAL`: http://www.gdal.org
.. _`BSD`: http://www.opensource.org/licenses/bsd-license.php
.. _`point cloud data`: http://en.wikipedia.org/wiki/Point_cloud
.. _`LIDAR`: http://en.wikipedia.org/wiki/LIDAR

