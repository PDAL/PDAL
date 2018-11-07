.. _readers.las:

readers.las
===========

The **LAS Reader** supports reading from `LAS format`_ files, the standard
interchange format for LIDAR data.  The reader does NOT support point formats
containing waveform data (4, 5, 9 and 10).

The reader also supports compressed LAS files, known as LAZ files or
`LASzip`_ files.
In order to use compresed LAS (LAZ), your version of PDAL must be built
with one of the two supported decompressors, `LASzip`_ or `LAZperf`_.
See the :ref:`compression <las_compression>` option below for more information.

.. _LASzip: http://www.laszip.org
.. _LAZperf: https://github.com/verma/laz-perf

.. note::

  LAS stores X, Y and Z dimensions as scaled integers.  Users converting an
  input LAS file to an output LAS file will frequently want to use the same
  scale factors and offsets in the output file as existed in the input
  file in order to
  maintain the precision of the data.  Use the `forward` option on the
  :ref:`writers.las` to facilitate transfer of header information from
  source to destination LAS/LAZ files.

.. note::

  LAS 1.4 files can contain datatypes that are actually arrays rather than
  individual dimensions.  Since PDAL doesn't support these datatypes, it
  must map them into datatypes it supports.  This is done by appending the
  array index to the name of the datatype.  For example, datatypes 11 - 20
  are two dimensional array types and if a field had the name Foo for
  datatype 11, PDAL would create the dimensions Foo0 and Foo1 to hold the
  values associated with LAS field Foo.  Similarly, datatypes 21 - 30 are
  three dimensional arrays and a field of type 21 with the name Bar would
  cause PDAL to create dimensions Bar0, Bar1 and Bar2.  See the information
  on the extra bytes VLR in the `LAS Specification`_ for more information
  on the extra bytes VLR and array datatypes.

.. warning::

  LAS 1.4 files that use the extra bytes VLR and datatype 0 will be accepted,
  but the data associated with a dimension of datatype 0 will be ignored
  (no PDAL dimension will be created).

.. embed::

.. streamable::


Example
-------

.. code-block:: json
    :linenos:

    {
      "pipeline":[
        {
          "type":"readers.las",
          "filename":"inputfile.las"
        },
        {
          "type":"writers.text",
          "filename":"outputfile.txt",
        }
      ]
    }

Options
-------

_`filename`
  LAS file to read [Required]

.. include:: reader_opts.rst

_`extra_dims`
  Extra dimensions to be read as part of each point beyond those specified by
  the LAS point format.  The format of the option is
  <dimension_name>=<type>, ... where type is one of:
  int8, int16, int32, int64, uint8, uint16, uint32, uint64, float, double.
  `_t` may be added to any of the type names as well (e.g., uint32_t).

  .. note::

      The presence of an extra bytes VLR when reading a version
      1.4 file or a version 1.0 - 1.3 file with **use_eb_vlr** set
      causes this option to be ignored.

.. _LAS format: http://asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html
.. _LAS Specification: http://www.asprs.org/a/society/committees/standards/LAS_1_4_r13.pdf

_`use_eb_vlr`
  If an extra bytes VLR is found in a version 1.0 - 1.3 file, use it as if it
  were in a 1.4 file. This option has no effect when reading a version 1.4 file.
  [Default: false]

.. _las_compression:

compression
  May be set to "lazperf" or "laszip" to choose either the LazPerf decompressor
  or the LASzip decompressor for LAZ files.  PDAL must have been built with
  support for the decompressor being requested.  The LazPerf decompressor
  doesn't support version 1 LAZ files or version 1.4 of LAS. [Default: 'none']

