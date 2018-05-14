.. _readers.pcd:

******************************************************************************
readers.pcd
******************************************************************************


The **PCD Reader** supports reading from `Point Cloud Data (PCD)`_ formatted
files, which are used by the `Point Cloud Library (PCL)`_.

.. note::

    The `PCD Reader` requires linkage of the `PCL`_ library.

.. plugin::


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
          "type":"writers.text",
          "filename":"outputfile.txt"
        }
      ]
    }

Options
-------

filename
  PCD file to read [Required]

count
  Maximum number of points to read [Optional]

.. _Point Cloud Data (PCD): http://pointclouds.org/documentation/tutorials/pcd_file_format.php
.. _Point Cloud Library (PCL): http://pointclouds.org
.. _PCL: http://pointclouds.org

