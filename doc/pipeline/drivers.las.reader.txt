.. _drivers.las.reader:

drivers.las.reader
==================

The **LAS Reader** supports reading from `LAS format`_ files, the standard interchange format for LIDAR data.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.text.writer">
      <Option name="filename">outputfile.txt</Option>
      <Reader type="drivers.las.reader">
        <Option name="filename">inputfile.las</Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  LAS file to read [Required] 



.. _LAS format: http://asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html
  
