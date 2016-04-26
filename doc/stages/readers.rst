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

