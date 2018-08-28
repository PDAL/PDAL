.. _readers.nitf:

readers.nitf
============

The `NITF`_ format is used primarily by the US Department of Defense and
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

    Only LAS or LAZ data may be stored in the LIDARA segment. PDAL uses
    the :ref:`readers.las` and :ref:`writers.las`
    to actually read and write the data.

.. note::

    PDAL uses a fork of the `NITF Nitro`_ library available at
    https://github.com/hobu/nitro for NITF read and write support.

.. _`NITF Nitro`: http://nitro-nitf.sourceforge.net/wikka.php?wakka=HomePage

.. embed::

.. streamable::


Example
-------

.. code-block:: json
    :linenos:

    {
      "pipeline":[
        {
          "type":"readers.nitf",
          "filename":"mynitf.nitf"
        },
        {
          "type":"writers.las",
          "filename":"outputfile.las"
        }
      ]
    }


Options
-------

filename
  Filename to read from [Required]

count
  Maximum number of points to read [Optional]

spatialreference
  Spatial reference to apply to data

extra_dims
  Dimensions to assign to extra byte data

compression
  May be set to "lazperf" or "laszip" to choose either the LazPerf decompressor
  or the LASzip decompressor for LAZ files.  PDAL must have been built with
  support for the decompressor being requested.  The LazPerf decompressor
  doesn't support version 1 LAZ files or version 1.4 of LAS.
  [Default: "laszip"]

.. _NITF: http://en.wikipedia.org/wiki/National_Imagery_Transmission_Format

.. _NITF 2.1: http://www.gwg.nga.mil/ntb/baseline/docs/2500c/index.html

.. _DES segment: http://jitc.fhu.disa.mil/cgi/nitf/registers/desreg.aspx
