.. _writers.bpf:

writers.bpf
===========

BPF is an NGA specification for point cloud data. The specification can be
found at https://nsgreg.nga.mil/doc/view?i=4202 The PDAL **BPF Writer**
only supports writing of version 3 BPF format files.

Example
-------


.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.",s
          "filename":"inputfile.las"
        },
        {
          "type":"writers.bpf",
          "filename":"outputfile.bpf"
        }
      ]
    }

Options
-------

filename
    BPF file to read.  The writer will accept a filename containing
    a single placeholder character ('#').  If input to the writer consists
    of multiple PointViews, each will be written to a separate file, where
    the placeholder will be replaced with an incrementing integer.  If no
    placeholder is found, all PointViews provided to the writer are
    aggregated into a single file for output.  Multiple PointViews are usually
    the result of using :ref:`filters.splitter`, :ref:`filters.chipper` or
    :ref:`filters.divider`.
    [Required]

compression
    This option can be set to true to cause the file to be written with Zlib
    compression as described in the BPF specification.  [Default: false]

format
    Specifies the format for storing points in the file. [Default: dim]

    * dim == Dimension-major (non-interleaved).  All data for a single dimension
      are stored contiguously.
    * point == Point-major (interleaved).  All data for a single point
      are stored contiguously.
    * byte == Byte-major (byte-segregated).  All data for a single dimension are
      stored contiguously, but bytes are arranged such that the first bytes for
      all points are stored contiguously, followed by the second bytes of all
      points, etc.  See the BPF specification for further information.

bundledfile
    Path of file to be written as a bundled file (see specification).  The path
    part of the filespec is removed and the filename is stored as part of the
    data.  This option can be specified as many times as desired.

header_data
    Base64-encoded data that will be decoded and written following the
    standard BPF header.

coord_id
    The coordinate ID (UTM zone) of the data.  NOTE: Only the UTM coordinate
    type is currently supported. [Default: 0, with coordinate type set to none]

scale_x, scale_y, scale_z
    Scale to be divided from the X, Y and Z nominal values, respectively, after
    the offset has been applied.  The special value "auto" can be specified,
    which causes the writer to select a scale to set the stored values of the
    dimensions to range from [0, 2147483647].  [Default: .01]

    .. note::

        written value = (nominal value - offset) / scale.

offset_x, offset_y, offset_z
    Offset to be subtracted from the X, Y and Z nominal values, respectively,
    before the value is scaled.  The special value "auto" can be specified,
    which causes the writer to set the offset to the minimum value of the
    dimension.  [Default: auto]

    .. note::

        written value = (nominal value - offset) / scale.

    .. note::

        Because BPF data is always stored in UTM, the XYZ offsets are set to
        "auto" by default. This is to avoid truncation of the decimal digits
        (which may occur with offsets left at 0).

output_dims
    If specified, limits the dimensions written for each point.  Dimensions
    are listed by name and separated by commas.  X, Y and Z are required and
    must be explicitly listed.
