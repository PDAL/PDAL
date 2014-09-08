.. _drivers.pcd.writer:

drivers.pcd.writer
==================

The **PCD Writer** supports writing to `Point Cloud Data (PCD)`_ formatted
files, which are used by the `Point Cloud Library (PCL)`_.

By default, compression is not enabled, and the PCD writer will output ASCII
formatted data. When compression is enabled, the output is PCD's
binary-compressed format.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.pcd.writer">
      <Option name="filename">outputfile.pcd</Option>
      
      <Reader type="drivers.pcd.reader">
        <Option name="filename">inputfile.pcd</Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  PCD file to write [Required] 

compression
  Apply compression to the PCD file? [Default: false]
  


.. _Point Cloud Data (PCD): http://pointclouds.org/documentation/tutorials/pcd_file_format.php
.. _Point Cloud Library (PCL): http://pointclouds.org
 
