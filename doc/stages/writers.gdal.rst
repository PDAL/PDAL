.. _writers.gdal:

writers.gdal
================================================================================

The **GDAL writer** creates a raster from a point cloud using an interpolation
algorithm.  Output is produced using `GDAL`_ and can use any `driver
that supports creation of rasters`_.  A data_type_ can be specified for the
raster (double, float, int32, etc.).  If no data type is specified, the
data type with the largest range supported by the driver is used.

.. _`GDAL`: http://gdal.org
.. _`driver that supports creation of rasters`: http://www.gdal.org/formats_list.html

The technique used to create the raster is a simple interpolation where
each point that falls within a given radius_ of a raster cell center
potentially contributes to the raster's value.  If no radius is provided,
it is set to the product of the resolution_ and the square root of two.
If a circle with the provided radius
doesn't encompass the entire cell, it is possible that some points will
not be considered at all, including those that may be within the bounds
of the raster cell.

The GDAL writer creates rasters using the data specified in the dimension_
option (defaults to `Z`). The writer creates up to six rasters based on
different statistics in the output dataset.  The order of the layers in the
dataset is as follows:

min
    Give the cell the minimum value of all points within the given radius.

max
    Give the cell the maximum value of all points within the given radius.

mean
    Give the cell the mean value of all points within the given radius.

idw
    Cells are assigned a value based on `Shepard's inverse distance weighting`_
    algorithm, considering all points within the given radius.

count
    Give the cell the number of points that lie within the given radius.

stdev
    Give the cell the population standard deviation of the points that lie
    within the given radius.

.. _`Shepard's inverse distance weighting`: https://en.wikipedia.org/wiki/Inverse_distance_weighting

If no points fall within the circle about a raster cell, a secondary
algorithm can be used to attempt to provide a value after the standard
interpolation is complete.  If the window_size_ option is non-zero, the
values of a square of rasters surrounding an empty cell is applied
using inverse distance weighting of any non-empty cells.
The value provided for window_size is the
maximum horizontal or vertical distance that a donor cell may be in order to
contribute to the subject cell (A window_size of 1 essentially creates a 3x3
array around the subject cell.  A window_size of 2 creates a 5x5 array, and
so on.)

Cells that have no value after interpolation are given a value specified by
the nodata_ option.

.. embed::

.. streamable::


Basic Example
--------------------------------------------------------------------------------

This  pipeline reads the file autzen_trim.las and creates a Geotiff dataset
called outputfile.tif.  Since output_type isn't specified, it creates six
raster bands ("min", "max", "mean", "idx", "count" and "stdev") in the output
dataset.  The raster cells are 10x10 and the radius used to locate points
whose values contribute to the cell value is 14.14.

.. code-block:: json

  [
      "pdal/test/data/las/autzen_trim.las",
      {
          "resolution": 10,
          "radius": 14.14,
          "filename":"outputfile.tif"
      }
  ]


Options
--------------------------------------------------------------------------------

filename
    Name of output file. The writer will accept a filename containing
    a single placeholder character (`#`).  If input to the writer consists
    of multiple PointViews, each will be written to a separate file, where
    the placeholder will be replaced with an incrementing integer.  If no
    placeholder is found, all PointViews provided to the writer are
    aggregated into a single file for output.  Multiple PointViews are usually
    the result of using :ref:`filters.splitter`, :ref:`filters.chipper` or
    :ref:`filters.divider`.[Required]

.. _resolution:

resolution
    Length of raster cell edges in X/Y units.  [Required]

.. _radius:

radius
    Radius about cell center bounding points to use to calculate a cell value.
    [Default: resolution_ * sqrt(2)]

power
    Exponent of the distance when computing IDW. Close points have higher
    significance than far points. [Default: 1.0]

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

.. _data_type:

data_type
    The :ref:`data type <types>` to use for the output raster.
    Many GDAL drivers only
    support a limited set of output data types.
    [Default: depends on the driver]

.. _nodata:

nodata
    The value to use for a raster cell if no data exists in the input data
    with which to compute an output cell value. [Default: depends on the
    data_type_.  -9999 for double, float, int and short, 9999 for
    unsigned int and unsigned short, 255 for unsigned char and -128 for char]

.. _output_type:

output_type
    A comma separated list of statistics for which to produce raster layers.
    The supported values are "min", "max", "mean", "idw", "count", "stdev"
    and "all".  The option may be specified more than once. [Default: "all"]

.. _window_size:

window_size
    The maximum distance from a donor cell to a target cell when applying
    the fallback interpolation method.  See the stage description for more
    information. [Default: 0]

.. _dimension:

dimension
  A dimension name to use for the interpolation. [Default: "Z"]

bounds
  The bounds of the data to be written.  Points not in bounds are discarded.
  The format is ([minx, maxx],[miny,maxy]). [Optional]

origin_x
  X origin (lower left corner) of the grid. [Default: None]

origin_y
  Y origin (lower left corner) of the grid. [Default: None]

width
  Number of cells in the X direction. [Default: None]

height
  Number of cells in the Y direction. [Default: None]

.. note::
    You may use the 'bounds' option, or 'origin_x', 'origin_y', 'width'
    and 'height', but not both.
