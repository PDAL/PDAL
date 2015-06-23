.. _writers.bpf:

writers.bpf
===========

The **BPF Writer** supports writing of version 3 BPF format files.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.bpf">
      <Option name="filename">
        outputfile.bpf
      </Option>
      <Reader type="readers.las">
        <Option name="filename">
          inputfile.las
        </Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  BPF file to read [Required] 

compression
  This option can be set to true to cause the file to be written with Zlib
  compression as described in the BPF specification.  [Default: false]

format
  Specifies the format for storing points in the file. [Default: dim]

  * dim == Dimension-major (non-interleaved).  All data for a single dimension
    are stored contiguously.
  * point == Point-major (interleaved).  All data for a single point are stored
    contiguously.
  * byte == Byte-major (byte-segregated).  All data for a single dimension are
    stored contiguously, but bytes are arranged such that the first bytes for
    all points are stored contiguously, followed by the second bytes of all
    points, etc.  See the BPF specification for further information.

coord_id
  The coordinate ID (UTM zone) of the data.  NOTE: Only the UTM coordinate
  type is currently supported. [Default: 0, with coordinate type set to none]

scale_x, scale_y, scale_z
  Scale to be divided from the X, Y and Z nominal values, respectively, after
  the offset has been applied.  The special value "auto" can be specified,
  which causes the writer to select a scale to set the stored values of the
  dimensions to range from [0, 2147483647].  [Default: .01]

  Note: written value = (nominal value - offset) / scale.

offset_x, offset_y, offset_z
   Offset to be subtracted from the X, Y and Z nominal values, respectively,
   before the value is scaled.  The special value "auto" can be specified,
   which causes the writer to set the offset to the minimum value of the
   dimension.  [Default: 0]

   Note: written value = (nominal value - offset) / scale.
