.. _writers.nitf:

writers.nitf
============

The `NITF`_ format is a US Department of Defense format for the transmission
of imagery.  It supports various formats inside a generic wrapper.  However,
the PDAL NITF writer only supports LAS/LAZ data.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.nitf">
      <Option name="filename">mynitf.nitf</Option>
      <Option name="ONAME">James Alexander</Option>
      <Option name="OPHONE">646-322-3123</Option>
      <Reader type="readers.las">
        <Option name="filename">inputfile.las</Option>
      </Reader>
    </Writer>
  </Pipeline>


Options
-------

filename
  NITF file to write.  The writer will accept a filename containing
  a single placeholder character ('#').  If input to the writer consists
  of multiple PointViews, each will be written to a separate file, where
  the placeholder will be replaced with an incrementing integer.  If no
  placeholder is found, all PointViews provided to the writer are
  aggregated into a single file for output.  Multiple PointViews are usually
  the result of using :ref:`filters.splitter` or :ref:`filters.chipper`.

CLEVEL
  File complexity level (2 characters) [Default: **03**]
  
STYPE
  Standard type (4 characters) [Default: **BF01**]

OSTAID
  Originating station ID (10 characters) [Default: **PDAL**]

FTITLE
  File title (80 characters) [Default: <spaces>]

FSCLAS
  File security classification ('T', 'S', 'C', 'R' or 'U') [Default: **U**]

ONAME
  Originator name (24 characters) [Default: <spaces>]

OPHONE
  Originator phone (18 characters) [Default: <spaces>]

FSCTLH
  File control and handling (2 characters) [Default: <spaces>]

FSCLSY
  File classification system (2 characters) [Default: <spaces>]

IDATIM
  Image date and time (format: 'CCYYMMDDhhmmss') [Default: <spaces>]

IID2
  Image identifier 2 (80 characters) [Default: <spaces>]

FSCLTX
  File classification text (43 characters) [Default: <spaces>]

AIMIDB
  Option tag that should contain further options to complete the AIMIDB
  (Additional Image ID) TRE record [Default: None]

ACFTB
  Option tag that should contain further options to complete the ACFTB
  (Aircraft Information) TRE record [Default: None]


.. _NITF: http://en.wikipedia.org/wiki/National_Imagery_Transmission_Format

.. _NITF 2.1: http://www.gwg.nga.mil/ntb/baseline/docs/2500c/index.html

.. _DES segment: http://jitc.fhu.disa.mil/cgi/nitf/registers/desreg.aspx
