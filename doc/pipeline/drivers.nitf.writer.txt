.. _drivers.nitf.writer:

drivers.nitf.writer
===================

The `NITF`_ format is used primarily by the US Department of Defence and supports many kinds of data inside a generic wrapper. The `NITF 2.1`_ version added support for LIDAR point cloud data, and the **NITF file writing** supports reading that data.

The dimensions read by the writer match should exactly to the LAS dimension names and types for convenience in file format transformation.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.nitf.writer">
      <Option name="filename">mynitf.nitf</Option>
      <Option name="ONAME">James Alexander</Option>
      <Option name="OPHONE">646-322-3123</Option>
      <Reader type="drivers.las.reader">
        <Option name="filename">inputfile.las</Option>
      </Reader>
    </Writer>
  </Pipeline>


Options
-------

filename
  Filename to read from [Required] 

CLEVEL
  Set the "compliance level" metadata [Default: **03**]
  
STYPE
  Set the "system type" metadata [Default: **BF01**]

OSTAID
  Set the "origin station id" metadata [Default: **PDAL**]

FTITLE
  Set the "file title" metadata [Default: **FTITLE**]

FSCLAS
  Set the "classification" metadata [Default: **U**]

ONAME
  Set the "originator name" metadata [Default: none]

OPHONE
  Set the "originator phone" metadata [Default: none]

IDATIM
  Set the "file date" metadata [Default: none]





.. _NITF: http://en.wikipedia.org/wiki/National_Imagery_Transmission_Format

.. _NITF 2.1: http://www.gwg.nga.mil/ntb/baseline/docs/2500c/index.html

.. _DES segment: http://jitc.fhu.disa.mil/cgi/nitf/registers/desreg.aspx