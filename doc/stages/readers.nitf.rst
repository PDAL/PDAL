.. _readers.nitf:

readers.nitf
============

The `NITF`_ format is used primarily by the US Department of Defence and
supports many kinds of data inside a generic wrapper. The `NITF 2.1`_ version
added support for LIDAR point cloud data, and the **NITF file reader** supports
reading that data, if the NITF file supports it.

* The file must be NITF 2.1
* There must be at least one Image segment ("IM").
* There must be at least one `DES segment`_ ("DE") named "LIDARA".
* Only LAS or LAZ data may be stored in the LIDARA segment

The dimensions produced by the reader match exactly to the LAS dimension names
and types for convenience in file format transformation.

.. note::
    
    Only LAS or LAZ data may be stored in the LIDARA segment

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">outputfile.las</Option>
      <Reader type="readers.nitf">
        <Option name="filename">mynitf.nitf</Option>
      </Reader>
    </Writer>
  </Pipeline>


Options
-------

filename
  Filename to read from [Required]



.. _NITF: http://en.wikipedia.org/wiki/National_Imagery_Transmission_Format

.. _NITF 2.1: http://www.gwg.nga.mil/ntb/baseline/docs/2500c/index.html

.. _DES segment: http://jitc.fhu.disa.mil/cgi/nitf/registers/desreg.aspx
