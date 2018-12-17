.. _readers.bpf:

******************************************************************************
readers.bpf
******************************************************************************

BPF is an NGA specification for point cloud data. The specification can be
found at https://nsgreg.nga.mil/doc/view?i=4220&month=8&day=30&year=2016 The **BPF Reader** supports
reading from BPF files that are encoded as version 1, 2 or 3.

This BPF reader only supports Zlib compression.  It does NOT support the
deprecated compression types QuickLZ and FastLZ.  The reader will consume files
containing ULEM frame data and polarimetric data, although these data are not
made accessible to PDAL; they are essentially ignored.

Data that follows the standard header but precedes point data is taken to
be metadata and is UTF-encoded and added to the reader's metadata.

.. embed::

.. streamable::

Example
------------------------------------------------------------------------------

.. code-block:: json

    {
      "pipeline":[
        "inputfile.bpf",
        {
          "type":"writers.text",
          "filename":"outputfile.txt"
        }
      ]
    }


Options
------------------------------------------------------------------------------

filename
    BPF file to read [Required]

.. include:: reader_opts.rst

