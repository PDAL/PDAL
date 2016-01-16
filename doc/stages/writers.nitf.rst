.. _writers.nitf:

writers.nitf
============

The `NITF`_ format is a US Department of Defense format for the transmission
of imagery.  It supports various formats inside a generic wrapper.

.. note::

    LAS inside of NITF is widely supported by software that uses NITF
    for point cloud storage, and LAZ is supported by some softwares.
    No other content type beyond those two is widely supported as
    of January of 2016.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.nitf">
      <Option name="filename">mynitf.nitf</Option>
      <Option name="oname">James Alexander</Option>
      <Option name="ophone">646-322-3123</Option>
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
  the result of using :ref:`filters.splitter`, :ref:`filters.chipper` or
  :ref:`filters.divider`.

clevel
  File complexity level (2 characters) [Default: **03**]

stype
  Standard type (4 characters) [Default: **BF01**]

ostaid
  Originating station ID (10 characters) [Default: **PDAL**]

ftitle
  File title (80 characters) [Default: <spaces>]

fsclas
  File security classification ('T', 'S', 'C', 'R' or 'U') [Default: **U**]

oname
  Originator name (24 characters) [Default: <spaces>]

ophone
  Originator phone (18 characters) [Default: <spaces>]

fsctlh
  File control and handling (2 characters) [Default: <spaces>]

fsclsy
  File classification system (2 characters) [Default: <spaces>]

idatim
  Image date and time (format: 'CCYYMMDDhhmmss') [Default: <spaces>]

iid2
  Image identifier 2 (80 characters) [Default: <spaces>]

fscltx
  File classification text (43 characters) [Default: <spaces>]

aimidb
  Comma separated list of name/value pairs to complete the AIMIDB
  (Additional Image ID) TRE record (format name:value) [Default: None]

acftb
  Comma separated list of name/value pairs to complete the ACFTB
  (Aircraft Information) TRE record (format name:value) [Default: None]


.. _NITF: http://en.wikipedia.org/wiki/National_Imagery_Transmission_Format

.. _NITF 2.1: http://www.gwg.nga.mil/ntb/baseline/docs/2500c/index.html

.. _DES segment: http://jitc.fhu.disa.mil/cgi/nitf/registers/desreg.aspx
