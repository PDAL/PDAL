.. _readers:

Readers
=======

Readers provide :ref:`dimensions` to :ref:`pipeline`. PDAL attempts to
normalize common dimension types, like X, Y, Z, or Intensity, which are often
found in LiDAR point clouds. Not all dimension types need to be fixed, however.
Database drivers typically return unstructured lists of dimensions.  A reader
might provide a simple file type, like :ref:`readers.text`, a complex database
like :ref:`readers.oci`, or a network service like :ref:`readers.greyhound`.


.. toctree::
   :maxdepth: 1
   :glob:

   readers.*

