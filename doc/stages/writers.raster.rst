.. _writers.raster:

writers.raster
================================================================================

The **Raster Writer** writes an existing raster to a file.
Output is produced using `GDAL`_ and can use any `driver
that supports creation of rasters`_.  A data_type_ can be specified for the
raster (double, float, int32, etc.).  If no data type is specified, the
data type with the largest range supported by the driver is used.

.. _`GDAL`: http://gdal.org
.. _`driver that supports creation of rasters`: http://www.gdal.org/formats_list.html

Cells that have no value are given a value specified by the nodata_ option.

.. embed::

.. streamable::


Basic Example
--------------------------------------------------------------------------------

This  pipeline reads the file autzen_trim.las, triangulates the data, creates a raster
based on the `Z` dimension as determined by interpolation of the location and values
of 'Z' of the vertices of a containing triangle, if any exists.  The resulting raster
is written to "outputfile.tif".

.. code-block:: json

  [
      "pdal/test/data/las/autzen_trim.las",
      {
          "type": "filters.delaunay"
      }
      {
          "type": "filters.faceraster",
          "resolution": 1
      }
      {
          "type": "writers.raster"
          "filename":"outputfile.tif"
      }
  ]


Options
--------------------------------------------------------------------------------

filename
    Name of output file. [Required]

.. _resolution:

gdaldriver
    GDAL code of the `GDAL driver`_ to use to write the output.
    [Default: "GTiff"]

.. _`GDAL driver`: http://www.gdal.org/formats_list.html

gdalopts
    A list of key/value options to pass directly to the GDAL driver.  The
    format is name=value,name=value,...  The option may be specified
    any number of times.

    .. note::
        The INTERLEAVE GDAL driver option is not supported.  writers.gdal
        always uses BAND interleaving.

rasters
    A comma-separated list of raster names to be written as bands of the raster.
    All rasters must have the same limits (origin/width/height). Rasters following the first
    that don't have the same limits will be dropped. If no raster names are provided,
    only the first raster found will be placed into a single band for output.

.. _data_type:

data_type
    The :ref:`data type <types>` to use for the output raster.  Many GDAL drivers only
    support a limited set of output data types.  [Default: depends on the driver]

.. _nodata:

nodata
    The value to use for a raster cell if the raster contains no data in a cell.
    Note that the nodata written to the output may be different from that of the
    raster being written.
    [Default: depends on the data_type_.  -9999 for double, float, int and short, 9999 for
    unsigned int and unsigned short, 255 for unsigned char and -128 for char]

.. include:: writer_opts.rst
