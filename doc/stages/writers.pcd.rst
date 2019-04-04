.. _writers.pcd:

writers.pcd
===========

The **PCD Writer** supports writing to `Point Cloud Data (PCD)`_ formatted
files, which are used by the `Point Cloud Library (PCL)`_.

By default, compression is not enabled, and the PCD writer will output ASCII
formatted data. When compression is enabled, the output is PCD's
binary-compressed format.

.. plugin::

.. note::

    The `PCD Writer` requires linkage of the `PCL`_ library.


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
  Level of PCD compression to use (ascii, binary, compressed)
  [Default: "ascii"]

xyz
  Write only XYZ dimension? [Default: "false"]

subtract_minimum
  Set origin to minimum of XYZ dimension? [Default: true]

offset_x, offset_y, offset_z
  Offset to be subtracted from XYZ position [Default: 0.0]

scale_x, scale_y, scale_z
  Scale to divide from XYZ dimension [Default: 1.0]

.. _Point Cloud Data (PCD): http://pointclouds.org/documentation/tutorials/pcd_file_format.php
.. _Point Cloud Library (PCL): http://pointclouds.org
.. _PCL: http://pointclouds.org

