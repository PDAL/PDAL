.. _readers.las:

readers.las
===========

The **LAS Reader** supports reading from `LAS format`_ files, the standard interchange format for LIDAR data.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.text">
      <Option name="filename">outputfile.txt</Option>
      <Reader type="readers.las">
        <Option name="filename">inputfile.las</Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  LAS file to read [Required] 



.. _LAS format: http://asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html
  
