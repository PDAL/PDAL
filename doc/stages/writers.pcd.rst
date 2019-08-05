.. _writers.pcd:

writers.pcd
===========

The **PCD Writer** supports writing to `Point Cloud Data (PCD)`_ formatted
files, which are used by the `Point Cloud Library (PCL)`_.

By default, compression is not enabled, and the PCD writer will output ASCII
formatted data.

.. embed::

.. streamable::


Example
-------

.. code-block:: json

  [
      {
          "type":"readers.pcd",
          "filename":"inputfile.pcd"
      },
      {
          "type":"writers.pcd",
          "filename":"outputfile.pcd"
      }
  ]

Options
-------

filename
  PCD file to write [Required]

compression
  Level of PCD compression to use (ascii, binary, compressed) [Default:
  "ascii"]

_`precision`
  Decimal Precision for output of values. This can be overridden for individual
  dimensions using the order option. [Default: 3]

_`order`
  Comma-separated list of dimension names in the desired output order. For
  example "X,Y,Z,Red,Green,Blue". Dimension names can optionally be followed
  by a PDAL type (e.g., Unsigned32) and dimension-specific precision (used only
  with "ascii" compression).  Ex: "X=Float:2, Y=Float:2, Z=Float:3,
  Intensity=Unsigned32" If no precision is specified the value provided with
  the precision_ option is used.  The default dimension type is double
  precision float. [Default: none]

keep_unspecified
  If true, writes all dimensions. Dimensions specified with the order_ option
  precede those not specified. [Default: **true**]


.. _Point Cloud Data (PCD): http://pointclouds.org/documentation/tutorials/pcd_file_format.php
.. _Point Cloud Library (PCL): http://pointclouds.org

