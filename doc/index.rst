.. _home:

******************************************************************************
PDAL - Point Data Abstraction Library
******************************************************************************

.. image:: ./_static/pdal_logo.png
   :alt: PDAL logo
   :align: right

PDAL is a C++ library for translating and manipulating `point cloud
data`_.  It is very much like the `GDAL`_ library which handles raster and
vector data.  The :ref:`about` page provides high level overview of the library
and its philosophy. Visit :ref:`readers` and :ref:`writers` to list data
formats it supports, and see :ref:`filters` for filtering operations that you
can apply with PDAL.

In addition to the library code, PDAL provides a suite of command-line
applications that users can conveniently use to process, filter, translate, and
query point cloud data.  :ref:`apps` provides more information on that topic.

Finally, PDAL speaks Python by both embedding and extending it. Visit
:ref:`python` to find out how you can use PDAL with Python to process point
cloud data.

The entire website is available as a single PDF at http://pdal.io/PDAL.pdf

News
--------------------------------------------------------------------------------

**09-09-2020**
................................................................................

PDAL 2.2.0 has been released. You can :ref:`download <download>` the source
code or follow the :ref:`quickstart <quickstart>` to get going in a
hurry with Conda.



.. _`Howard Butler`: http://github.com/hobu
.. _`Brad Chambers`: http://github.com/chambbj

About
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2

   about


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
   stages/stages
   stages/readers
   stages/writers
   stages/filters

Dimensions
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2

   dimensions

Types
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2

   types

Python
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2

   python

Java
--------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 2

   java

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
   project/index
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

