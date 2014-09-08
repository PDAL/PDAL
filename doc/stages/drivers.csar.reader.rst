.. _drivers.csar.reader:

drivers.csar.reader
===================

The **CSAR file reader** supports reading from `CARIS spatial archive files`_. 

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.text.writer">
      <Option name="filename">outputfile.txt</Option>
      <Reader type="drivers.csar.reader">
        <Option name="filename">mycsar.csar</Option>
      </Reader>
    </Writer>
  </Pipeline>


Options
-------

filename
  Filename to read from [Required] 


.. _CARIS spatial archive files: http://en.wikipedia.org/wiki/Caris_Spatial_Archive
  
