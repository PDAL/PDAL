.. _writers.pcd:

writers.pcd
===========

The **PCD Writer** supports writing to `Point Cloud Data (PCD)`_ formatted
files, which are used by the `Point Cloud Library (PCL)`_.

By default, compression is not enabled, and the PCD writer will output ASCII
formatted data. When compression is enabled, the output is PCD's
binary-compressed format.

.. note::

    The `PCD Writer` requires linkage of the `PCL`_ library.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.pcd",
          "filename":"inputfile.pcd"
        },
        {
          "type":"writers.pcd",
          "filename":"outputfile.pcd"
        }
      ]
    }

Options
-------

filename
  PCD file to write [Required]

compression
  Apply compression to the PCD file? [Default: false]



.. _Point Cloud Data (PCD): http://pointclouds.org/documentation/tutorials/pcd_file_format.php
.. _Point Cloud Library (PCL): http://pointclouds.org
.. _PCL: http://pointclouds.org

