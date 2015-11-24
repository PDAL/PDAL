.. _stage_index:

******************************************************************************
Readers, Writers, and Filters
******************************************************************************

The :cpp:class:`pdal::Stage` object encapsulates the reading, writing, and
filtering capabilities for PDAL.

.. _readers:

Readers
=======

Readers provide data to :ref:`pipeline` operations. Readers might be a simple
file type, like :ref:`readers.las`, a complex database like :ref:`readers.oci`, or
a network service like :ref:`readers.greyhound`.

.. note::

    Readers provide :cpp:class:`pdal::Dimension` to :ref:`pipeline`. PDAL attempts
    to normalize common dimension types, like X, Y, Z, or Intensity, which are
    often found in LiDAR point clouds. Not all dimension types need to be fixed, however.
    Database drivers typically return unstructured lists of dimensions.

.. toctree::
   :maxdepth: 1
   :glob:

   readers.*

.. _writers:

Writers
=======

Writers consume data provided by :ref:`readers`. Some writers can consume any
dimension type, while others only understand fixed dimension names.

.. note::

    Some writers can only consume known dimension names, while PDAL doesn't
    yet have a registery for the dimension types, you can see the
    base dimension types at https://github.com/PDAL/PDAL/blob/master/include/pdal/Dimension.hpp

.. toctree::
   :maxdepth: 1
   :glob:

   writers.*

.. _filters:

Filters
=======

Filters operate on data as inline operations. They can remove, modify, reorganize,
and add points to the data stream as it goes by. Some filters can only operate on
dimensions they understand (consider :ref:`filters.reprojection` doing geographic
reprojection on XYZ coordinates), while others do not interrogate the point data at
all and simply reorganize or split data.

.. toctree::
   :maxdepth: 1
   :glob:

   filters.*
