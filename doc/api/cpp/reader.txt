.. _cpp-reader:

******************************************************************************
:cpp:class:`pdal::Reader`
******************************************************************************

:cpp:class:`pdal::Reader` are classes that provided interfaces to various the
various point cloud formats and hands them off to a PDAL pipeline in a common
format that is described via the :cpp:class:`pdal::Schema`.

.. doxygenclass:: pdal::Reader
   :members:
   :sections: public*


`ASPRS LAS`_ Reader
------------------------------------------------------------------------------

The `ASPRS LAS`_ format is a sequential binary format used to store data from
LiDAR sensors and by LiDAR processing software for data interchange and archival.


.. _`ASPRS LAS`: http://www.asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html

.. doxygenclass:: pdal::drivers::las::Reader
   :members:

OCI Reader
------------------------------------------------------------------------------

The OCI reader provides `Oracle Point Cloud`_ support to PDAL.

.. _`Oracle Point Cloud`: http://docs.oracle.com/cd/B28359_01/appdev.111/b28400/sdo_pc_pkg_ref.htm

.. doxygenclass:: pdal::drivers::oci::Reader
   :members:

QFIT Reader
------------------------------------------------------------------------------
`QFIT`_ is a format for the NASA IceBridge sea ice sensor.
 
.. _`QFIT`: http://nsidc.org/data/ilatm1b.html

.. doxygenclass:: pdal::drivers::qfit::Reader
  :members:

TerraSolid Reader
------------------------------------------------------------------------------

.. doxygenclass:: pdal::drivers::terrasolid::Reader
 :members:


