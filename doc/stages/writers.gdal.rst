.. _writers.gdal:

writers.gdal
================================================================================

The `GDAL`_ writer creates a raster from a point cloud using an interpolation
algorithm.  Output is produced using `GDAL`_ and can therefore use any `driver
that supports creation of rasters`_.

.. _`GDAL`: http://gdal.org
.. _`driver that supports creation of rasters`: http://www.gdal.org/formats_list.html

The technique used to create the raster is a simple interpolation where
each point that falls within a given radius of a raster cell center
potentially contributes to the raster's value.

.. note::

    If a circle with the provided radius doesn't encompass the entire cell,
    it is possible that some points will not be considered at all, including
    those that may be within the bounds of the raster cell.

The GDAL writer creates rasters using the data in the 'Z' dimension of the 
points.  One can precede the GDAL writer with the ferry filter
(:ref:`filters.ferry`) in order to copy data from some other dimension to
the 'Z' dimension if desired.

The writer will creates up to six rasters based on different statistics in
the output dataset.  The order of the layers in the dataset is as follows:

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
interpolation is complete.  If the window_size option is set to a non-zero
value, a square of rasters surrounding an empty cell, and the value of each
non-empty surrounding is averaged using inverse distance weighting to provide
a value for the subject cell.  The value provided for :ref:`window_size` is the
horizontal or vertical distance that a donor cell may be in order to
contribute to the subject cell (A window_size of 1 essentially creates a 3x3
array around the subject cell.  A window_size of 2 creates a 5x5 array, and
so on.)

Cells that have no value after interpolation are given the empty value of -9999.

Basic Example
--------------------------------------------------------------------------------

.. code-block:: json

    {
      "pipeline":[
        "pdal/test/data/las/autzen_trim.las",
        {
          "edge_length", 10,
          "radius", 14.14,
          "filename":"outputfile.tif"
        }
      ]
    }


Options
--------------------------------------------------------------------------------

filename
    Name of output file. [Required]

edge_length
    Length of raster cell edges.  [Required]

radius
    Radius about cell center bounding points to use to calculate a cell value.

gdaldriver
    Name of the GDAL driver to use to write the output. [Default: "GTiff"]

gdalopts
    A list of key/value options to pass directly to the GDAL driver.  The
    format is name=value,name=value,...  The option may be specified
    any number of times.

output_type
    A comma separated list of statistics for which to produce raster layers.
    The supported values are "min", "max", "mean", "idw", "count", "stdev"
    and "all".  The options may be specified more than once. [Default: "all"]

window_size
    The maximum distance from a donor cell to a target cell when applying
    a fallback interpolation method.  See the stage description for more
    information. [Default: 0]

